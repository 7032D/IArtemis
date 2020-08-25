/*****************************************************************************/
/*!
 * @file     ./dcmc_rv_motor.c
 * @version  $Rev: 4703 $
 * @date     Creation    : 05/05/2011
 * @date     Last commit : $Date: 2015-01-13 10:56:11 +0100 (Tue, 13 Jan 2015) $
 * @date     Modified    :
 * @author   Xavier RAVE (c)
 * @project  EDRES (Environ. Develop. Robotique Exploration Spatiale)
 * @system   DRVPR
 * @brief    Rover Motor Control
 ***************************************************************************/
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <limits.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <math.h>
#include <assert.h>
#include "dcmc_rv_motor_typ.h"
#include "dcmc_rv_motor_head.h"

/*--------------------------------*/ /*---------------------*/
/*          GLOSSARY              */ /*         API         */
/*--------------------------------*/ /*    -------------    */
/*  rv  = rover private function  */ /*    | dcmc_*    |    */
/*  dmc = driving motor card      */ /*    -------------    */
/*  smc = steering motor card     */ /*    |   rv_*    |    */
/*  lw  = left wheel              */ /*    -------------    */
/*  rw  = right wheel             */ /*    | dmc | smc |    */
/*  lrw = left and right wheels   */ /*    -------------    */
/*  ls  = left side               */ /*    | bit R/W   |    */
/*  rs  = right side              */ /*    -------------    */
/*--------------------------------*/ /*---------------------*/

/* Driving motor card axes go left and right */
/*
      |----------|
    --|  L    R  |--
      |----------|
*/
/* Steering motor card axes go down */
/*
      |----------|
      |   o  o   |
      |----------|
*/

/*----------------------------------------------------*/
/*        iartemis                          ARTEMIS      */
/*----------------------------------------------------*/
/*
      |----------|
    --|  Lo  oR  |--                -------  -------
      |----------|                --|  R  |  |  L  |--
                                    |  o  |  |  o  |
                                    |     |  |     |
      |----------|                  |  o  |  |  o  |
    --|  Lo  oR  |--              --|  L  |  |  R  |--
      |----------|                  -------  ------


      |----------|                    |----------|
    --|  Lo  oR  |--                --|  Lo  oR  |--
      |----------|                    |----------|
*/
/*----------------------------------------------------*/

/*---------------*/
/*     RULES     */
/*---------------*/
/*
 1. S_ROVER struct must not be modified directly by user of this API

 2. Each public dcmc_rv_* function is applied (written) immediately on cards

 3. IDLE state = all motors turned off

 4. If speed is > 0 left side goes forward, what ever the configuration is
    If speed is > 0 right side goes forward, except in CIRCLE configuration

 5. If speed is < 0 left side goes backward, what ever the configuration is
    If speed is < 0 right side goes backward, except in CIRCLE configuration

 6. If driving speed sign changed, a pause is needed

 7. Parameter control is done only on public functions

 8. A function who modify parameters without applying them must set b_modified flag

 9. Multi threading
    circle   = 1 thread
    straight = 1 thread
    crab     = 2 threads, main + steering control thread
    tank     = 1-3 threads, main + [left side thread] + [right side thread]

10. Reading of FILE is done with fscanf, it doesn't work with fgets.

11. FILE must be opened with fopen and "a+" mode.

*/

/*-------------*/
/*     TODO    */
/*-------------*/
/*
 1. Add Artmemis rover (P1) => FAIT.
 2. Check mutex usage, protect file access (P1)
 4. Management of b_modified at card level (P3)
 5. Improve sleep management, use time after each command, check time before sleeping (P2)
 6. Rewrite functions with left ou right parameter (ok pour smc, to be checked for dmc - pointer on vte1/vte2) (P3)
10. Add a mid parameter in smc (P1) => FAIT.
11. Ignore calls with value identical to previous value (P2)
12. Check what happens when calling set_config during a dmc ls ou rs sleep (P1)
13. When there is a signed change in tank mode, update with the last received value (P1)
14. Use a thread to store last received value in straight or circle mode when sleeping (P2)
15. Add a close function (P1) => FAIT.
16. Implement STRAIGHT_ROT mode with set_speed in tank mode (P1)
17. Add a test mode (P1)
18. Check if it's possible to send a synchro on one card only (P2)
19. Check in smc that min steering is right and max is left (P1) => FAIT.
    => On artemis min is left and max is right !
    => On iartemis min is right and max is left !
20. Smc and Dmc debug functions can be rewritten with new param field (P3)
21. Use ftd2xx API instead of /dev/ttyUSB0 (P3) :
    FT_Open, FT_ResetDevice, FT_Purge, FT_SetFlowControl, FT_SetBaudRate, FT_SetDataCharacteristics
    FT_SetChars, FT_SetLatencyTimer, FT_GetQueueStatus, FT_Write, FT_Read
22. Ajouter une butee software (on ajoute une marge lors de la definition des min/max) (P1) => FAIT.
    On conserve ainsi la precision pour mettre en position middle
23. Mettre un mode arret urgence (emergency stop) = envoi un arret de tous les moteurs (P1) => FAIT.
24. Gerer une liste de rover pour le handler SIGINT
25. Revoir la gestion des boolens (prefixe e_ b_ i_)
*/

/**
 * @addtogroup dcmc_private
 * @{
 * @name Rover motor controller API
 * @{
 */

/* Static pointer to rover for SIGINT handler */
static S_ROVER *gps_rover;
static T_RM_BOOLEAN gb_emergencystop;

/*-----------------*/
/* Error functions */
/*-----------------*/
static const char *tpc_error[] = {"OK", "Null Parameter", "Invalid Parameter", "Invalid Configuration", "Invalid State",
                                  "Read Error", "Write Error", "Thread Error", "Mutex Error", "Timeout Error"};

#define PRINT_ERROR(e_errstat)                                                                                         \
    if (e_errstat)                                                                                                     \
    fprintf(stderr, "Error in %s:%d => %s\n", __FILE__, __LINE__, tpc_error[(e_errstat)])

/*-------------------------*/
/* Unsigned char functions */
/*-------------------------*/

static const unsigned char tuc_bit_0[8] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F};
static const unsigned char tuc_bit_1[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

static inline unsigned char uchar_set_bit_0(unsigned char uc_value, int i_bit_pos) {
    return (uc_value & tuc_bit_0[i_bit_pos]);
}

static inline unsigned char uchar_set_bit_1(unsigned char uc_value, int i_bit_pos) {
    return (uc_value | tuc_bit_1[i_bit_pos]);
}

static unsigned char uchar_set_bit(unsigned char uc_value, int i_bit_pos, int i_bit_value) {
    unsigned char uc_tmp;
    switch (i_bit_value) {
        case 0:
            uc_tmp = uchar_set_bit_0(uc_value, i_bit_pos);
            break;
        case 1:
            uc_tmp = uchar_set_bit_1(uc_value, i_bit_pos);
            break;
        default:
            uc_tmp = uc_value;
            break;
    }
    return uc_tmp;
}

static inline unsigned char uchar_reverse_bit(unsigned char uc_value, int i_bit_pos) {
    unsigned char uc_tmp;
    uc_tmp = uc_value & tuc_bit_1[i_bit_pos];
    return (uc_value ^ uc_tmp);
}

static inline int uchar_is_bit_set(unsigned char uc_value, int i_bit_pos) {
    return (uc_value & tuc_bit_1[i_bit_pos]);
}

static unsigned char uchar_get_from_pct(int i_value_pct) {
    if (i_value_pct > 100) {
        i_value_pct = 100;
    } else if (i_value_pct < -100) {
        i_value_pct = -100;
    }
    return (unsigned char)(abs(i_value_pct) * UCHAR_MAX / 100);
}

/*------------------------*/
/* Driving CARD functions */
/*------------------------*/
static inline void dmc_init(S_RM_DMC *ps_dmc) {
    ps_dmc->c_ident = 'x';
    ps_dmc->s_wr_buff.uc_comen1 = 0x00; /* ! brake on */
    ps_dmc->s_wr_buff.uc_comen2 = 0x00; /* ! brake on */
    ps_dmc->s_wr_buff.uc_razen = 0x00;
    ps_dmc->s_wr_buff.uc_vte1 = 0x00;
    ps_dmc->s_wr_buff.uc_vte2 = 0x00;
}

static inline void dmc_set_address(char c_address, S_RM_DMC *ps_dmc) {
    ps_dmc->c_ident = c_address;
}

inline void dmc_set_lw_speed(unsigned char uc_speed, S_RM_DMC *ps_dmc) {
    ps_dmc->s_wr_buff.uc_vte1 = uc_speed;
}

inline void dmc_set_rw_speed(unsigned char uc_speed, S_RM_DMC *ps_dmc) {
    ps_dmc->s_wr_buff.uc_vte2 = uc_speed;
}

static inline void dmc_set_lrw_speed(unsigned char uc_speed, S_RM_DMC *ps_dmc) {
    ps_dmc->s_wr_buff.uc_vte1 = uc_speed;
    ps_dmc->s_wr_buff.uc_vte2 = uc_speed;
}

static inline void dmc_set_lrw_gear(E_RM_GEAR e_gear, S_RM_DMC *ps_dmc) {
    ps_dmc->s_wr_buff.uc_comen2 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen2, 1, e_gear);
    ps_dmc->s_wr_buff.uc_comen2 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen2, 3, e_gear);
}

static inline void dmc_set_lrw_brake(E_RM_BRAKE e_brake, S_RM_DMC *ps_dmc) {
    ps_dmc->s_wr_buff.uc_comen1 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen1, 0, e_brake);
    ps_dmc->s_wr_buff.uc_comen1 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen1, 2, e_brake);
    ps_dmc->s_wr_buff.uc_comen2 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen2, 0, e_brake);
    ps_dmc->s_wr_buff.uc_comen2 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen2, 2, e_brake);
}

static inline void dmc_set_lw_on(T_RM_BOOLEAN e_boolean, S_RM_DMC *ps_dmc) {
    ps_dmc->s_wr_buff.uc_comen1 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen1, 5, e_boolean);
}

static inline void dmc_set_rw_on(T_RM_BOOLEAN e_boolean, S_RM_DMC *ps_dmc) {
    ps_dmc->s_wr_buff.uc_comen1 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen1, 7, e_boolean);
}

static inline void dmc_set_lrw_on(T_RM_BOOLEAN e_boolean, S_RM_DMC *ps_dmc) {
    ps_dmc->s_wr_buff.uc_comen1 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen1, 5, e_boolean);
    ps_dmc->s_wr_buff.uc_comen1 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen1, 7, e_boolean);
}

static inline void dmc_set_lw_dir(E_RM_DMC_DIR e_direction, S_RM_DMC *ps_dmc) {
    ps_dmc->s_wr_buff.uc_comen1 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen1, 4, e_direction);
}

static inline void dmc_set_rw_dir(E_RM_DMC_DIR e_direction, S_RM_DMC *ps_dmc) {
    ps_dmc->s_wr_buff.uc_comen1 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen1, 6, e_direction);
}

static inline void dmc_set_lrw_dir(E_RM_DMC_DIR e_direction, S_RM_DMC *ps_dmc) {
    ps_dmc->s_wr_buff.uc_comen1 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen1, 4, e_direction);
    ps_dmc->s_wr_buff.uc_comen1 = uchar_set_bit(ps_dmc->s_wr_buff.uc_comen1, 6, e_direction);
}

static E_RM_ERROR dmc_fprintf(FILE *ps_file, S_RM_DMC *ps_dmc) {
    E_RM_ERROR e_errstat;
    int i_status1;
    int i_status2;

    e_errstat = E_RM_ERROR_OK;
    if (ps_file == NULL || ps_dmc == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (!e_errstat) {
        i_status1 = fprintf(ps_file, "*%c>%02x%02x%02x%02x%02x\n", ps_dmc->c_ident, ps_dmc->s_wr_buff.uc_comen1,
                            ps_dmc->s_wr_buff.uc_comen2, ps_dmc->s_wr_buff.uc_razen, ps_dmc->s_wr_buff.uc_vte1,
                            ps_dmc->s_wr_buff.uc_vte2);
        i_status2 = fflush(ps_file);
        if (i_status1 < 0 || i_status2 != 0) {
            e_errstat = E_RM_ERROR_WRITE;
        }
    }
    return e_errstat;
}

// scan buffer from driving motor card
E_RM_ERROR dmc_fscanf(FILE *ps_file, S_RM_DMC *ps_dmc) {
    char tc_buffer[256];
    E_RM_ERROR e_errstat;
    int i_status1;
    int i_status2;
    unsigned int ui_seuilen;
    unsigned int ui_ntope13;
    unsigned int ui_ntope12;
    unsigned int ui_ntope11;
    unsigned int ui_ntope23;
    unsigned int ui_ntope22;
    unsigned int ui_ntope21;
    unsigned int ui_perie12;
    unsigned int ui_perie11;
    unsigned int ui_perie22;
    unsigned int ui_perie21;

    e_errstat = E_RM_ERROR_OK;
    if (ps_file == NULL || ps_dmc == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (I_RM_TRUE) {
        if (!e_errstat) {
            /* send synchro */
            i_status1 = fprintf(ps_file, "*x#80\n");
            i_status2 = fflush(ps_file);
            if (i_status1 < 0 || i_status2 != 0) {
                e_errstat = E_RM_ERROR_READ;
            }
        }
        if (!e_errstat) {
            /* send read request */
            usleep(1000);
            i_status1 = fprintf(ps_file, "*%c<\n", ps_dmc->c_ident);
            i_status2 = fflush(ps_file);
            if (i_status1 < -1 || i_status2 != 0) {
                e_errstat = E_RM_ERROR_READ;
            }
        }
        if (!e_errstat) {
            /* scan device file */
            i_status1 = fscanf(ps_file, "%s", tc_buffer);
            if (i_status1 != 1) {
                e_errstat = E_RM_ERROR_READ;
            }
        }
        if (!e_errstat) {
            /* scan data and update read buffer */
            i_status1 = sscanf(tc_buffer, "!%*c<%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", &ui_seuilen, &ui_ntope13,
                               &ui_ntope12, &ui_ntope11, &ui_ntope23, &ui_ntope22, &ui_ntope21, &ui_perie12,
                               &ui_perie11, &ui_perie22, &ui_perie21);
            /*fprintf(stdout, "%s\n", tc_buffer);*/
            if (i_status1 == 11) { /* scan 11 values */
                ps_dmc->s_rd_buff.uc_seuilen = (unsigned char)ui_seuilen;
                ps_dmc->s_rd_buff.uc_ntope13 = (unsigned char)ui_ntope13;
                ps_dmc->s_rd_buff.uc_ntope12 = (unsigned char)ui_ntope12;
                ps_dmc->s_rd_buff.uc_ntope11 = (unsigned char)ui_ntope11;
                ps_dmc->s_rd_buff.uc_ntope23 = (unsigned char)ui_ntope23;
                ps_dmc->s_rd_buff.uc_ntope22 = (unsigned char)ui_ntope22;
                ps_dmc->s_rd_buff.uc_ntope21 = (unsigned char)ui_ntope21;
                ps_dmc->s_rd_buff.uc_perie12 = (unsigned char)ui_perie12;
                ps_dmc->s_rd_buff.uc_perie11 = (unsigned char)ui_perie11;
                ps_dmc->s_rd_buff.uc_perie22 = (unsigned char)ui_perie22;
                ps_dmc->s_rd_buff.uc_perie21 = (unsigned char)ui_perie21;
            } else {
                e_errstat = E_RM_ERROR_READ;
            }
        }
    }
    return e_errstat;
}

static void dmc_left_debug(FILE *ps_file, S_RM_DMC *ps_dmc) {
    unsigned char uc_comen1;
    unsigned char uc_comen2;
    unsigned char uc_razen;
    unsigned char uc_vte1;
    unsigned char uc_vte2;
    if (ps_dmc != NULL && ps_file != NULL) {
        uc_comen1 = ps_dmc->s_wr_buff.uc_comen1;
        uc_comen2 = ps_dmc->s_wr_buff.uc_comen2;
        uc_razen = ps_dmc->s_wr_buff.uc_razen;
        uc_razen = uc_razen; /* to suppress warning */
        uc_vte1 = ps_dmc->s_wr_buff.uc_vte1;
        uc_vte1 = uc_vte1; /* to suppress warning */
        uc_vte2 = ps_dmc->s_wr_buff.uc_vte2;
        uc_vte2 = uc_vte2; /* to suppress warning */
        /* left */
        fprintf(ps_file, "[");
        uchar_is_bit_set(uc_comen1, 5) ? fprintf(ps_file, "ON ") : fprintf(ps_file, "OFF ");
        uchar_is_bit_set(uc_comen1, 4) ? fprintf(ps_file, "DIR:CCW ") : fprintf(ps_file, "DIR:CW  ");
        fprintf(ps_file, "SPEED:%03d ", uc_vte1);
        uchar_is_bit_set(uc_comen2, 1) ? fprintf(ps_file, "GEAR:HIGH ") : fprintf(ps_file, "GEAR:LOW ");
        uchar_is_bit_set(uc_comen1, 0) ? fprintf(ps_file, "EBRAKE:OFF ") : fprintf(ps_file, "EBRAKE:ON ");
        uchar_is_bit_set(uc_comen2, 0) ? fprintf(ps_file, "MBRAKE:OFF") : fprintf(ps_file, "MBRAKE:ON");
        fprintf(ps_file, "] ");
        fprintf(ps_file, "[ %d %d] ", (int)ps_dmc->s_rd_buff.uc_ntope11, (int)ps_dmc->s_rd_buff.uc_perie11);
        fflush(ps_file);
    }
}

static void dmc_right_debug(FILE *ps_file, S_RM_DMC *ps_dmc) {
    unsigned char uc_comen1;
    unsigned char uc_comen2;
    unsigned char uc_razen;
    unsigned char uc_vte1;
    unsigned char uc_vte2;
    if (ps_dmc != NULL && ps_file != NULL) {
        uc_comen1 = ps_dmc->s_wr_buff.uc_comen1;
        uc_comen2 = ps_dmc->s_wr_buff.uc_comen2;
        uc_razen = ps_dmc->s_wr_buff.uc_razen;
        uc_razen = uc_razen;
        uc_vte1 = ps_dmc->s_wr_buff.uc_vte1;
        uc_vte1 = uc_vte1;
        uc_vte2 = ps_dmc->s_wr_buff.uc_vte2;
        uc_vte2 = uc_vte2;
        /* right */
        fprintf(ps_file, "[");
        uchar_is_bit_set(uc_comen1, 7) ? fprintf(ps_file, "ON ") : fprintf(ps_file, "OFF ");
        uchar_is_bit_set(uc_comen1, 6) ? fprintf(ps_file, "DIR:CCW ") : fprintf(ps_file, "DIR:CW  ");
        fprintf(ps_file, "SPEED:%03d ", uc_vte2);
        uchar_is_bit_set(uc_comen2, 3) ? fprintf(ps_file, "GEAR:HIGH ") : fprintf(ps_file, "GEAR:LOW ");
        uchar_is_bit_set(uc_comen1, 2) ? fprintf(ps_file, "EBRAKE:OFF ") : fprintf(ps_file, "EBRAKE:ON ");
        uchar_is_bit_set(uc_comen2, 2) ? fprintf(ps_file, "MBRAKE:OFF") : fprintf(ps_file, "MBRAKE:ON");
        fprintf(ps_file, "] ");
        fprintf(ps_file, "[ %d %d] ", (int)ps_dmc->s_rd_buff.uc_ntope21, (int)ps_dmc->s_rd_buff.uc_perie21);
        fflush(ps_file);
    }
}

static void dmc_debug(FILE *ps_file, S_RM_DMC *ps_dmc) {
    dmc_left_debug(ps_file, ps_dmc);
    dmc_right_debug(ps_file, ps_dmc);
}

/*-------------------------*/
/* Steering CARD functions */
/*-------------------------*/

// initialise steering motor card
static inline void smc_init(S_RM_SMC *ps_smc) {
    ps_smc->c_ident = 'x';
    ps_smc->b_read = I_RM_TRUE;
    ps_smc->b_simu = I_RM_FALSE;
    ps_smc->s_wr_buff.uc_comax = 0x00;
    ps_smc->s_wr_buff.uc_razax = 0x00;
    ps_smc->s_wr_buff.uc_slogi = 0x00;
    ps_smc->s_rd_buff.uc_agld1 = 0x7F;
    ps_smc->s_rd_buff.uc_agld2 = 0x7F;
    ps_smc->s_params.uc_lw_agl_min = 0x7F;
    ps_smc->s_params.uc_lw_agl_mid = 0x7F;
    ps_smc->s_params.uc_lw_agl_max = 0x7F;
    ps_smc->s_params.uc_rw_agl_min = 0x7F;
    ps_smc->s_params.uc_rw_agl_mid = 0x7F;
    ps_smc->s_params.uc_rw_agl_max = 0x7F;
    ps_smc->s_params.b_lw_on = I_RM_FALSE;          /* = 0 */
    ps_smc->s_params.b_rw_on = I_RM_FALSE;          /* = 0 */
    ps_smc->s_params.e_lw_dir = E_RM_SMC_DIR_RIGHT; /* = 0 */
    ps_smc->s_params.e_rw_dir = E_RM_SMC_DIR_RIGHT; /* = 0 */
    ps_smc->s_params.uc_lw_agl_ord = 0x7F;
    ps_smc->s_params.uc_rw_agl_ord = 0x7F;
}

// set address of steering motor card
static inline void smc_set_address(char c_address, S_RM_SMC *ps_smc) {
    ps_smc->c_ident = c_address;
}

// set left wheel direction of steering motor card
static inline void smc_set_lw_dir(E_RM_SMC_DIR e_direction, S_RM_SMC *ps_smc) {
    ps_smc->s_params.e_lw_dir = e_direction;
    ps_smc->s_wr_buff.uc_comax = uchar_set_bit(ps_smc->s_wr_buff.uc_comax, 2, e_direction);
}

// set right wheel direction of steering motor card
static inline void smc_set_rw_dir(E_RM_SMC_DIR e_direction, S_RM_SMC *ps_smc) {
    ps_smc->s_params.e_rw_dir = e_direction;
    ps_smc->s_wr_buff.uc_comax = uchar_set_bit(ps_smc->s_wr_buff.uc_comax, 4, e_direction);
}

// set left and rigth wheels direction of steering motor card
/*static inline void smc_set_lrw_dir(E_RM_SMC_DIR e_direction, S_RM_SMC *ps_smc){*/
/*ps_smc->s_params.e_lw_dir  = e_direction;*/
/*ps_smc->s_params.e_rw_dir  = e_direction;*/
/*ps_smc->s_wr_buff.uc_comax = uchar_set_bit(ps_smc->s_wr_buff.uc_comax, 2, e_direction);*/
/*ps_smc->s_wr_buff.uc_comax = uchar_set_bit(ps_smc->s_wr_buff.uc_comax, 4, e_direction);*/
/*}*/

// set left wheel angle order of steering motor card
/*static inline void smc_set_lw_agl(unsigned char uc_angle, S_RM_SMC *ps_smc){*/
/*if( uc_angle >= ps_smc->s_params.uc_lw_agl_min && uc_angle <= ps_smc->s_params.uc_lw_agl_max ){*/
/*ps_smc->s_params.uc_lw_agl_ord = uc_angle;*/
/*}*/
/*}*/

// set right wheel angle order of steering motor card
/*static inline void smc_set_rw_agl(unsigned char uc_angle, S_RM_SMC *ps_smc){*/
/*if( uc_angle >= ps_smc->s_params.uc_rw_agl_min && uc_angle <= ps_smc->s_params.uc_rw_agl_max ){*/
/*ps_smc->s_params.uc_rw_agl_ord = uc_angle;*/
/*}*/
/*}*/

// set left wheel min and max values of steering motor card
static inline void smc_set_lw_agl_minmidmax(unsigned char uc_angle_min, unsigned char uc_angle_mid,
                                            unsigned char uc_angle_max, S_RM_SMC *ps_smc) {
    ps_smc->s_params.uc_lw_agl_min = uc_angle_min;
    ps_smc->s_params.uc_lw_agl_mid = uc_angle_mid;
    ps_smc->s_params.uc_lw_agl_max = uc_angle_max;
}

// set right wheel min and max values of steering motor card
static inline void smc_set_rw_agl_minmidmax(unsigned char uc_angle_min, unsigned char uc_angle_mid,
                                            unsigned char uc_angle_max, S_RM_SMC *ps_smc) {
    ps_smc->s_params.uc_rw_agl_min = uc_angle_min;
    ps_smc->s_params.uc_rw_agl_mid = uc_angle_mid;
    ps_smc->s_params.uc_rw_agl_max = uc_angle_max;
}

// set direction for going from min to max values of steering motor card
static inline void smc_set_mintomax_dir(E_RM_SMC_DIR e_mintomax_dir, S_RM_SMC *ps_smc) {
    ps_smc->s_params.e_mintomax_dir = e_mintomax_dir;
}

// set left wheel position of steering motor card
static void smc_set_lw_pos(E_RM_SMC_POS e_steering_position, S_RM_SMC *ps_smc) {
    switch (e_steering_position) {
        case E_RM_SMC_POS_LEFT:
            if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT) {
                ps_smc->s_params.uc_lw_agl_ord = ps_smc->s_params.uc_lw_agl_min;
            }
            if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT) {
                ps_smc->s_params.uc_lw_agl_ord = ps_smc->s_params.uc_lw_agl_max;
            }
            break;
        case E_RM_SMC_POS_MID:
            ps_smc->s_params.uc_lw_agl_ord = ps_smc->s_params.uc_lw_agl_mid;
            break;
        case E_RM_SMC_POS_RIGHT:
            if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT) {
                ps_smc->s_params.uc_lw_agl_ord = ps_smc->s_params.uc_lw_agl_max;
            }
            if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT) {
                ps_smc->s_params.uc_lw_agl_ord = ps_smc->s_params.uc_lw_agl_min;
            }
            break;
    }
}

// set right wheel position of steering motor card
static void smc_set_rw_pos(E_RM_SMC_POS e_steering_position, S_RM_SMC *ps_smc) {
    switch (e_steering_position) {
        case E_RM_SMC_POS_LEFT:
            if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT) {
                ps_smc->s_params.uc_rw_agl_ord = ps_smc->s_params.uc_rw_agl_min;
            }
            if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT) {
                ps_smc->s_params.uc_rw_agl_ord = ps_smc->s_params.uc_rw_agl_max;
            }
            break;
        case E_RM_SMC_POS_MID:
            ps_smc->s_params.uc_rw_agl_ord = ps_smc->s_params.uc_rw_agl_mid;
            break;
        case E_RM_SMC_POS_RIGHT:
            if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT) {
                ps_smc->s_params.uc_rw_agl_ord = ps_smc->s_params.uc_rw_agl_max;
            }
            if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT) {
                ps_smc->s_params.uc_rw_agl_ord = ps_smc->s_params.uc_rw_agl_min;
            }
            break;
    }
}

// set left and right wheels position of steering motor card
static inline void smc_set_lrw_pos(E_RM_SMC_POS e_steering_position, S_RM_SMC *ps_smc) {
    smc_set_lw_pos(e_steering_position, ps_smc);
    smc_set_rw_pos(e_steering_position, ps_smc);
}

// set left wheel on
static inline void smc_set_lw_on(T_RM_BOOLEAN e_boolean, S_RM_SMC *ps_smc) {
    ps_smc->s_params.b_lw_on = e_boolean;
    ps_smc->s_wr_buff.uc_comax = uchar_set_bit(ps_smc->s_wr_buff.uc_comax, 3, e_boolean);
}

// set right wheel on
static inline void smc_set_rw_on(T_RM_BOOLEAN e_boolean, S_RM_SMC *ps_smc) {
    ps_smc->s_params.b_rw_on = e_boolean;
    ps_smc->s_wr_buff.uc_comax = uchar_set_bit(ps_smc->s_wr_buff.uc_comax, 5, e_boolean);
}

// set left and right wheels on
static inline void smc_set_lrw_on(T_RM_BOOLEAN e_boolean, S_RM_SMC *ps_smc) {
    ps_smc->s_params.b_lw_on = e_boolean;
    ps_smc->s_params.b_rw_on = e_boolean;
    ps_smc->s_wr_buff.uc_comax = uchar_set_bit(ps_smc->s_wr_buff.uc_comax, 3, e_boolean);
    ps_smc->s_wr_buff.uc_comax = uchar_set_bit(ps_smc->s_wr_buff.uc_comax, 5, e_boolean);
}

// returns true if left wheel is on
static inline T_RM_BOOLEAN smc_is_lw_on(S_RM_SMC *ps_smc) {
    return ps_smc->s_params.b_lw_on;
}

// returns true if right wheel is on
static inline T_RM_BOOLEAN smc_is_rw_on(S_RM_SMC *ps_smc) {
    return ps_smc->s_params.b_rw_on;
}

// returns true if left or right wheel is on
static int smc_is_lrw_on(S_RM_SMC *ps_smc) {
    return ps_smc->s_params.b_lw_on || ps_smc->s_params.b_rw_on;
}

// returns true if wheel position is ok
/*static T_RM_BOOLEAN smc_is_lw_agl_ok(S_RM_SMC *ps_smc){*/
/*T_RM_BOOLEAN    b_ok;*/
/*unsigned char   uc_agl_ord;*/
/*b_ok        = I_RM_FALSE;*/
/*uc_agl_ord  = ps_smc->s_params.uc_lw_agl_ord;*/
/*if( abs(uc_agl_ord - ps_smc->s_rd_buff.uc_agld1) < UC_RM_SMC_POS_MARGIN*/
/*|| uc_agl_ord <= ps_smc->s_params.uc_lw_agl_min*/
/*|| uc_agl_ord >= ps_smc->s_params.uc_lw_agl_max ){*/
/*b_ok = I_RM_TRUE;*/
/*}*/
/*return b_ok;*/
/*}*/

// returns true if wheel position is ok
/*static T_RM_BOOLEAN smc_is_rw_agl_ok(S_RM_SMC *ps_smc){*/
/*T_RM_BOOLEAN    b_ok;*/
/*unsigned char   uc_agl_ord;*/
/*b_ok        = I_RM_FALSE;*/
/*uc_agl_ord  = ps_smc->s_params.uc_rw_agl_ord;*/
/*if( abs(uc_agl_ord - ps_smc->s_rd_buff.uc_agld2) < UC_RM_SMC_POS_MARGIN*/
/*|| uc_agl_ord <= ps_smc->s_params.uc_rw_agl_min*/
/*|| uc_agl_ord >= ps_smc->s_params.uc_rw_agl_max ){*/
/*b_ok = I_RM_TRUE;*/
/*}*/
/*return b_ok;*/
/*}*/

// check wheels direction and position and stop if needed (protection for crab configuration)
static T_RM_BOOLEAN smc_crab_checkstop_lrw(S_RM_SMC *ps_smc) {
    T_RM_BOOLEAN b_apply;
    unsigned char uc_agld1_current;
    unsigned char uc_agld2_current;

    b_apply = I_RM_FALSE;
    uc_agld1_current = ps_smc->s_rd_buff.uc_agld1;
    uc_agld2_current = ps_smc->s_rd_buff.uc_agld2;
    if (ps_smc->s_params.b_lw_on && ps_smc->s_params.e_lw_dir == E_RM_SMC_DIR_RIGHT) {
        if ((ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT &&
             uc_agld1_current > (ps_smc->s_params.uc_lw_agl_max - UC_RM_SMC_POS_MARGIN)) ||
            (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT &&
             uc_agld1_current < (ps_smc->s_params.uc_lw_agl_min + UC_RM_SMC_POS_MARGIN))) {
            /* stop left motor */
            smc_set_lw_on(I_RM_FALSE, ps_smc);
            b_apply = I_RM_TRUE;
        }
    }
    if (ps_smc->s_params.b_lw_on && ps_smc->s_params.e_lw_dir == E_RM_SMC_DIR_LEFT) {
        if ((ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT &&
             uc_agld1_current < (ps_smc->s_params.uc_lw_agl_min + UC_RM_SMC_POS_MARGIN)) ||
            (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT &&
             uc_agld1_current > (ps_smc->s_params.uc_lw_agl_max - UC_RM_SMC_POS_MARGIN))) {
            /* stop left motor */
            smc_set_lw_on(I_RM_FALSE, ps_smc);
            b_apply = I_RM_TRUE;
        }
    }
    if (ps_smc->s_params.b_rw_on && ps_smc->s_params.e_rw_dir == E_RM_SMC_DIR_RIGHT) {
        if ((ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT &&
             uc_agld2_current > (ps_smc->s_params.uc_rw_agl_max - UC_RM_SMC_POS_MARGIN)) ||
            (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT &&
             uc_agld2_current < (ps_smc->s_params.uc_rw_agl_min + UC_RM_SMC_POS_MARGIN))) {
            /* stop right motor */
            smc_set_rw_on(I_RM_FALSE, ps_smc);
            b_apply = I_RM_TRUE;
        }
    }
    if (ps_smc->s_params.b_rw_on && ps_smc->s_params.e_rw_dir == E_RM_SMC_DIR_LEFT) {
        if ((ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT &&
             uc_agld2_current < (ps_smc->s_params.uc_rw_agl_min + UC_RM_SMC_POS_MARGIN)) ||
            (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT &&
             uc_agld2_current > (ps_smc->s_params.uc_rw_agl_max - UC_RM_SMC_POS_MARGIN))) {
            /* stop right motor */
            smc_set_rw_on(I_RM_FALSE, ps_smc);
            b_apply = I_RM_TRUE;
        }
    }
    return b_apply;
}

// auto set direction and power on/off motors when needed
static void smc_set_lrw_dir_on_from_pos(S_RM_SMC *ps_smc) {
    unsigned char uc_agld1_current;
    unsigned char uc_agld2_current;

    uc_agld1_current = ps_smc->s_rd_buff.uc_agld1;
    uc_agld2_current = ps_smc->s_rd_buff.uc_agld2;
    if (uc_agld1_current < (ps_smc->s_params.uc_lw_agl_ord - UC_RM_SMC_POS_MARGIN)) {
        if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT) {
            smc_set_lw_dir(E_RM_SMC_DIR_RIGHT, ps_smc);
        } else if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT) {
            smc_set_lw_dir(E_RM_SMC_DIR_LEFT, ps_smc);
        }
        smc_set_lw_on(I_RM_TRUE, ps_smc);
    } else if (uc_agld1_current > (ps_smc->s_params.uc_lw_agl_ord + UC_RM_SMC_POS_MARGIN)) {
        if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT) {
            smc_set_lw_dir(E_RM_SMC_DIR_LEFT, ps_smc);
        } else if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT) {
            smc_set_lw_dir(E_RM_SMC_DIR_RIGHT, ps_smc);
        }
        smc_set_lw_on(I_RM_TRUE, ps_smc);
    } else { /* stop left motor */
        smc_set_lw_on(I_RM_FALSE, ps_smc);
    }
    if (uc_agld2_current < (ps_smc->s_params.uc_rw_agl_ord - UC_RM_SMC_POS_MARGIN)) {
        if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT) {
            smc_set_rw_dir(E_RM_SMC_DIR_RIGHT, ps_smc);
        } else if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT) {
            smc_set_rw_dir(E_RM_SMC_DIR_LEFT, ps_smc);
        }
        smc_set_rw_on(I_RM_TRUE, ps_smc);
    } else if (uc_agld2_current > (ps_smc->s_params.uc_rw_agl_ord + UC_RM_SMC_POS_MARGIN)) {
        if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT) {
            smc_set_rw_dir(E_RM_SMC_DIR_LEFT, ps_smc);
        } else if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT) {
            smc_set_rw_dir(E_RM_SMC_DIR_RIGHT, ps_smc);
        }
        smc_set_rw_on(I_RM_TRUE, ps_smc);
    } else { /* stop right motor */
        smc_set_rw_on(I_RM_FALSE, ps_smc);
    }
}

// print buffer to steering motor card
static E_RM_ERROR smc_fprintf(FILE *ps_file, S_RM_SMC *ps_smc) {
    E_RM_ERROR e_errstat;
    int i_status1;
    int i_status2;

    e_errstat = E_RM_ERROR_OK;
    if (ps_file == NULL || ps_smc == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (!e_errstat) {
        /*fprintf(stdout, "*%c>%02x%02x%02x\n",*/
        /*ps_smc->c_ident,*/
        /*ps_smc->s_wr_buff.uc_comax,*/
        /*ps_smc->s_wr_buff.uc_razax,*/
        /*ps_smc->s_wr_buff.uc_slogi);*/
        /*i_status1 = 0;*/
        i_status1 = fprintf(ps_file, "*%c>%02x%02x%02x\n", ps_smc->c_ident, ps_smc->s_wr_buff.uc_comax,
                            ps_smc->s_wr_buff.uc_razax, ps_smc->s_wr_buff.uc_slogi);
        i_status2 = fflush(ps_file);
        if (i_status1 < 0 || i_status2 != 0) {
            e_errstat = E_RM_ERROR_WRITE;
        }
    }
    return e_errstat;
}

// scan buffer from steering motor card
static E_RM_ERROR smc_fscanf(FILE *ps_file, S_RM_SMC *ps_smc) {
    char tc_buffer[256];
    E_RM_ERROR e_errstat;
    int i_status1;
    int i_status2;
    unsigned int ui_seuilax;
    unsigned int ui_elogi1;
    unsigned int ui_aglp;
    unsigned int ui_agld1;
    unsigned int ui_agld2;
    unsigned int ui_agll;
    unsigned int ui_aglm;
    unsigned int ui_agln;
    unsigned int ui_cour1;
    unsigned int ui_cour2;

    e_errstat = E_RM_ERROR_OK;
    if (ps_file == NULL || ps_smc == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (ps_smc->b_read) {
        if (!e_errstat) {
            /* send synchro */
            i_status1 = fprintf(ps_file, "*x#80\n");
            i_status2 = fflush(ps_file);
            if (i_status1 < 0 || i_status2 != 0) {
                e_errstat = E_RM_ERROR_READ;
            }
        }
        if (!e_errstat) {
            /* send read request */
            usleep(1000);
            i_status1 = fprintf(ps_file, "*%c<\n", ps_smc->c_ident);
            i_status2 = fflush(ps_file);
            if (i_status1 < -1 || i_status2 != 0) {
                e_errstat = E_RM_ERROR_READ;
            }
        }
        if (!e_errstat) {
            /* scan device file */
            i_status1 = fscanf(ps_file, "%s", tc_buffer);
            if (i_status1 != 1) {
                e_errstat = E_RM_ERROR_READ;
            }
        }
        if (!e_errstat && !ps_smc->b_simu) {
            /* scan data and update read buffer */
            i_status1 = sscanf(tc_buffer, "!%*c<%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", &ui_seuilax, &ui_elogi1,
                               &ui_aglp, &ui_agld1, &ui_agld2, &ui_agll, &ui_aglm, &ui_agln, &ui_cour1, &ui_cour2);
            /*fprintf(stdout, "%s\n", tc_buffer);*/
            if (i_status1 == 10) { /* scan 10 values */
                ps_smc->s_rd_buff.uc_seuilax = (unsigned char)ui_seuilax;
                ps_smc->s_rd_buff.uc_elogi1 = (unsigned char)ui_elogi1;
                ps_smc->s_rd_buff.uc_aglp = (unsigned char)ui_aglp;
                ps_smc->s_rd_buff.uc_agld1 = (unsigned char)ui_agld1;
                ps_smc->s_rd_buff.uc_agld2 = (unsigned char)ui_agld2;
                ps_smc->s_rd_buff.uc_agll = (unsigned char)ui_agll;
                ps_smc->s_rd_buff.uc_aglm = (unsigned char)ui_aglm;
                ps_smc->s_rd_buff.uc_agln = (unsigned char)ui_agln;
                ps_smc->s_rd_buff.uc_cour1 = (unsigned char)ui_cour1;
                ps_smc->s_rd_buff.uc_cour2 = (unsigned char)ui_cour2;
            } else {
                e_errstat = E_RM_ERROR_READ;
            }
        }
    }
    if (ps_smc->b_simu) {
        if (ps_smc->s_params.b_lw_on) {
            if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT) {
                ps_smc->s_rd_buff.uc_agld1 += 3 - 6 * ps_smc->s_params.e_lw_dir;
            } else if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT) {
                ps_smc->s_rd_buff.uc_agld1 += 6 * ps_smc->s_params.e_lw_dir - 3;
            }
        }
        if (ps_smc->s_params.b_rw_on) {
            if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_RIGHT) {
                ps_smc->s_rd_buff.uc_agld2 += 3 - 6 * ps_smc->s_params.e_rw_dir;
            } else if (ps_smc->s_params.e_mintomax_dir == E_RM_SMC_DIR_LEFT) {
                ps_smc->s_rd_buff.uc_agld2 += 6 * ps_smc->s_params.e_rw_dir - 3;
            }
        }
    }
    return e_errstat;
}

static void smc_left_debug(FILE *ps_file, S_RM_SMC *ps_smc) {
    unsigned char uc_comax;
    if (ps_smc != NULL && ps_file != NULL) {
        uc_comax = ps_smc->s_wr_buff.uc_comax;
        /* left */
        fprintf(ps_file, "[");
        uchar_is_bit_set(uc_comax, 3) ? fprintf(ps_file, "ON  ") : fprintf(ps_file, "OFF ");
        uchar_is_bit_set(uc_comax, 2) ? fprintf(ps_file, "DIR:LEFT ") : fprintf(ps_file, "DIR:RIGHT");
        fprintf(ps_file, " POS:%3d ORD:%3d", ps_smc->s_rd_buff.uc_agld1, ps_smc->s_params.uc_lw_agl_ord);
        fprintf(ps_file, "] ");
    }
}

static void smc_right_debug(FILE *ps_file, S_RM_SMC *ps_smc) {
    unsigned char uc_comax;
    if (ps_smc != NULL && ps_file != NULL) {
        uc_comax = ps_smc->s_wr_buff.uc_comax;
        /* right */
        fprintf(ps_file, "[");
        uchar_is_bit_set(uc_comax, 5) ? fprintf(ps_file, "ON  ") : fprintf(ps_file, "OFF ");
        uchar_is_bit_set(uc_comax, 4) ? fprintf(ps_file, "DIR:LEFT ") : fprintf(ps_file, "DIR:RIGHT");
        fprintf(ps_file, " POS:%3d ORD:%3d", ps_smc->s_rd_buff.uc_agld2, ps_smc->s_params.uc_rw_agl_ord);
        fprintf(ps_file, "] ");
    }
}

// print debugging information from steering card
static void smc_debug(FILE *ps_file, S_RM_SMC *ps_smc) {
    smc_left_debug(ps_file, ps_smc);
    smc_right_debug(ps_file, ps_smc);
}

/*--------------------*/
/* BCB CARD functions */
/*--------------------*/
static E_RM_ERROR bcb_write(FILE *ps_file, const char *pc_string) {
    E_RM_ERROR e_errstat;
    int i_status;

    e_errstat = E_RM_ERROR_OK;
    (void)fprintf(stderr, "(%d) %s", fileno(ps_file), pc_string);
    (void)fflush(stderr);
    i_status = 0;
    i_status = fprintf(ps_file, "%s", pc_string);
    (void)fflush(ps_file);
    if (i_status != (int)strlen(pc_string)) {
        e_errstat = E_RM_ERROR_WRITE;
    }
    return e_errstat;
}

static E_RM_ERROR bcb_read(FILE *ps_file, char *tc_buffer, int i_buffsize) {
    E_RM_ERROR e_errstat;
    int i_status;
    int i_fd;
    fd_set o_fdset;
    char *pc_ret;
    struct timeval s_tv;

    e_errstat = E_RM_ERROR_OK;
    i_status = 0;
    i_fd = fileno(ps_file);
    pc_ret = NULL;
    s_tv.tv_sec = I_RM_BCB_INIT_DELAY_S;
    s_tv.tv_usec = 0;

    FD_ZERO(&o_fdset);
    FD_SET(i_fd, &o_fdset);
    i_status = select(i_fd + 1, &o_fdset, NULL, NULL, &s_tv);
    switch (i_status) {
        case -1:
            perror("select");
            e_errstat = E_RM_ERROR_READ;
            break;
        case 0:
            e_errstat = E_RM_ERROR_TIMEOUT;
            break;
        case 1:
            pc_ret = fgets(tc_buffer, i_buffsize, ps_file);
            break;
        default:
            e_errstat = E_RM_ERROR_READ;
            break;
    }
    if (!e_errstat) {
        if (pc_ret == NULL) {
            e_errstat = E_RM_ERROR_READ;
        } else {
            fprintf(stderr, "%s\n", tc_buffer);
        }
    }
    return e_errstat;
}

/*-------------------------*/
/* Private ROVER functions */
/*-------------------------*/

void rv_dmc_init(S_ROVER *ps_rover) {
    int i_idx;
    for (i_idx = 0; i_idx < I_RM_NB_DMC; i_idx++) {
        dmc_init(&(ps_rover->ts_dmc[i_idx]));
        dmc_set_address('h' + 2 * i_idx, &(ps_rover->ts_dmc[i_idx]));
    }
}

/* rover driving direction functions */
void rv_dmc_set_ls_dir(E_RM_DMC_DIR e_direction, S_ROVER *ps_rover) {
    E_RM_DMC_DIR e_direction_opp;
    if (e_direction == E_RM_DMC_DIR_POS) {
        e_direction_opp = E_RM_DMC_DIR_NEG;
    } else {
        e_direction_opp = E_RM_DMC_DIR_POS;
    }
    dmc_set_lw_dir(e_direction_opp, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
    dmc_set_lw_dir(e_direction_opp, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
    dmc_set_lw_dir(e_direction_opp, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));

    ps_rover->e_dmc_ls_dir = e_direction;
    ps_rover->b_dmc_modified = 1;
}

void rv_dmc_set_rs_dir(E_RM_DMC_DIR e_direction, S_ROVER *ps_rover) {
    dmc_set_rw_dir(e_direction, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
    dmc_set_rw_dir(e_direction, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
    dmc_set_rw_dir(e_direction, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));

    ps_rover->e_dmc_rs_dir = e_direction;
    ps_rover->b_dmc_modified = 1;
}

static void rv_dmc_set_dir(E_RM_DMC_DIR e_direction, S_ROVER *ps_rover) {
    rv_dmc_set_ls_dir(e_direction, ps_rover);
    rv_dmc_set_rs_dir(e_direction, ps_rover);
}

/* rover on functions */
void rv_dmc_set_ls_on(T_RM_BOOLEAN e_boolean, S_ROVER *ps_rover) {
    if (ps_rover->e_model != E_RM_iartemis) {
        dmc_set_lrw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_LEFT]));
        dmc_set_lw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_REAR]));
    } else {
        dmc_set_lw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
        dmc_set_lw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
        dmc_set_lw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));
    }
    ps_rover->b_dmc_ls_on = e_boolean;
    ps_rover->b_dmc_modified = 1;
}

void rv_dmc_set_rs_on(T_RM_BOOLEAN e_boolean, S_ROVER *ps_rover) {
    if (ps_rover->e_model != E_RM_iartemis) {
        dmc_set_lrw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_RIGHT]));
        dmc_set_rw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_REAR]));
    } else {
        dmc_set_rw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
        dmc_set_rw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
        dmc_set_rw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));
    }
    ps_rover->b_dmc_rs_on = e_boolean;
    ps_rover->b_dmc_modified = 1;
}

void rv_dmc_set_on(T_RM_BOOLEAN e_boolean, S_ROVER *ps_rover) {
    int i_idx;
    for (i_idx = 0; i_idx < I_RM_NB_DMC; i_idx++) {
        dmc_set_lrw_on(e_boolean, &(ps_rover->ts_dmc[i_idx]));
    }
    ps_rover->b_dmc_ls_on = e_boolean;
    ps_rover->b_dmc_rs_on = e_boolean;
    ps_rover->b_dmc_modified = 1;
}

void rv_dmc_set_brake(E_RM_BRAKE e_brake, S_ROVER *ps_rover) {
    int i_idx;
    for (i_idx = 0; i_idx < I_RM_NB_DMC; i_idx++) {
        dmc_set_lrw_brake(e_brake, &(ps_rover->ts_dmc[i_idx]));
    }
    ps_rover->e_brake = e_brake;
    ps_rover->b_dmc_modified = 1;
}

static void rv_dmc_set_gear(E_RM_GEAR e_gear, S_ROVER *ps_rover) {
    int i_idx;
    for (i_idx = 0; i_idx < I_RM_NB_DMC; i_idx++) {
        dmc_set_lrw_gear(e_gear, &(ps_rover->ts_dmc[i_idx]));
    }
    ps_rover->e_gear = e_gear;
    ps_rover->b_dmc_modified = 1;
}

/* rover speed function */
static void rv_dmc_set_ls_speed_uc(unsigned char uc_speed, S_ROVER *ps_rover) {
    if (ps_rover->e_model != E_RM_iartemis) {
        dmc_set_lrw_speed(uc_speed, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_LEFT]));
        dmc_set_lw_speed(uc_speed, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_REAR]));
    } else {
        dmc_set_lw_speed(uc_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
        dmc_set_lw_speed(uc_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
        dmc_set_lw_speed(uc_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));
    }
    ps_rover->uc_dmc_ls_speed = uc_speed;
    ps_rover->b_dmc_modified = 1;
}

static void rv_dmc_set_rs_speed_uc(unsigned char uc_speed, S_ROVER *ps_rover) {
    if (ps_rover->e_model != E_RM_iartemis) {
        dmc_set_lrw_speed(uc_speed, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_RIGHT]));
        dmc_set_rw_speed(uc_speed, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_REAR]));
    } else {
        dmc_set_rw_speed(uc_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
        dmc_set_rw_speed(uc_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
        dmc_set_rw_speed(uc_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));
    }
    ps_rover->uc_dmc_rs_speed = uc_speed;
    ps_rover->b_dmc_modified = 1;
}

static void rv_dmc_set_speed_uc(unsigned char uc_speed, S_ROVER *ps_rover) {
    int i_idx;
    for (i_idx = 0; i_idx < I_RM_NB_DMC; i_idx++) {
        dmc_set_lrw_speed(uc_speed, &(ps_rover->ts_dmc[i_idx]));
    }
    ps_rover->uc_dmc_ls_speed = uc_speed;
    ps_rover->uc_dmc_rs_speed = uc_speed;
    ps_rover->b_dmc_modified = 1;
}

/* Apply driving orders on card */
E_RM_ERROR rv_dmc_apply(S_ROVER *ps_rover) {
    int i_idx;
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    if (ps_rover->b_dmc_modified) {
        for (i_idx = 0; !e_errstat && i_idx < I_RM_NB_DMC; i_idx++) {
            e_errstat = dmc_fprintf(ps_rover->ps_file, &(ps_rover->ts_dmc[i_idx]));
        }
        ps_rover->b_dmc_modified = 0;
    }
    return e_errstat;
}

static void rv_smc_init(S_ROVER *ps_rover) {
    int i_idx;
    unsigned char uc_margin;

    uc_margin = UC_RM_SMC_ENDSTOP_MARGIN;
    for (i_idx = 0; i_idx < I_RM_NB_SMC; i_idx++) {
        smc_init(&(ps_rover->ts_smc[i_idx]));
        smc_set_address('i' + 2 * i_idx, &(ps_rover->ts_smc[i_idx]));
        if (ps_rover->e_model == E_RM_STDOUT || ps_rover->e_model == E_RM_TEST) {
            ps_rover->ts_smc[i_idx].b_read = I_RM_FALSE;
            ps_rover->ts_smc[i_idx].b_simu = I_RM_TRUE;
            smc_set_mintomax_dir(E_RM_SMC_DIR_RIGHT, &(ps_rover->ts_smc[i_idx]));
        }
        if (ps_rover->e_model == E_RM_iartemis) {
            smc_set_mintomax_dir(E_RM_SMC_DIR_LEFT, &(ps_rover->ts_smc[i_idx]));
        }
        if (ps_rover->e_model == E_RM_ARTEMIS) {
            smc_set_mintomax_dir(E_RM_SMC_DIR_RIGHT, &(ps_rover->ts_smc[i_idx]));
        }
    }
    if (ps_rover->e_model == E_RM_TEST) {  // Test reading on test card
        ps_rover->ts_smc[E_RM_CARD_1].b_read = I_RM_TRUE;
        /*ps_rover->ts_smc[E_RM_CARD_0].b_read = I_RM_TRUE;*/
    }
    if (ps_rover->e_model == E_RM_iartemis) {
        /* Calibration of steering angles, after iartemis maintenance in november 2014 */
        smc_set_lw_agl_minmidmax(85 + uc_margin, 141, 204 - uc_margin, &(ps_rover->ts_smc[I_RM_iartemis_CARD_FRONT]));
        smc_set_rw_agl_minmidmax(65 + uc_margin, 128, 191 - uc_margin, &(ps_rover->ts_smc[I_RM_iartemis_CARD_FRONT]));
        smc_set_lw_agl_minmidmax(122 + uc_margin, 181, 241 - uc_margin, &(ps_rover->ts_smc[I_RM_iartemis_CARD_MIDDLE]));
        smc_set_rw_agl_minmidmax(83 + uc_margin, 141, 206 - uc_margin, &(ps_rover->ts_smc[I_RM_iartemis_CARD_MIDDLE]));
        smc_set_lw_agl_minmidmax(42 + uc_margin, 101, 161 - uc_margin, &(ps_rover->ts_smc[I_RM_iartemis_CARD_REAR]));
        smc_set_rw_agl_minmidmax(49 + uc_margin, 110, 175 - uc_margin, &(ps_rover->ts_smc[I_RM_iartemis_CARD_REAR]));
    } else {
        /* Calibration of steering angles, ARTEMIS in juillet 2012 */
        smc_set_lw_agl_minmidmax(168 + uc_margin, (168 + 242) / 2, 242 - uc_margin,
                                 &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_LEFT]));
        smc_set_rw_agl_minmidmax(164 + uc_margin, (164 + 237) / 2, 237 - uc_margin,
                                 &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_LEFT]));
        smc_set_lw_agl_minmidmax(154 + uc_margin, (154 + 229) / 2, 229 - uc_margin,
                                 &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_RIGHT]));
        smc_set_rw_agl_minmidmax(145 + uc_margin, (145 + 219) / 2, 219 - uc_margin,
                                 &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_RIGHT]));
        smc_set_lw_agl_minmidmax(146 + uc_margin, (146 + 219) / 2, 219 - uc_margin,
                                 &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_REAR]));
        smc_set_rw_agl_minmidmax(152 + uc_margin, (152 + 227) / 2, 227 - uc_margin,
                                 &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_REAR]));
    }
}

static void rv_smc_set_on(T_RM_BOOLEAN e_boolean, S_ROVER *ps_rover) {
    int i_idx;
    if (ps_rover != NULL) {
        for (i_idx = 0; i_idx < I_RM_NB_SMC; i_idx++) {
            smc_set_lrw_on(e_boolean, &(ps_rover->ts_smc[i_idx]));
        }
        ps_rover->b_smc_on = e_boolean;
        ps_rover->b_smc_modified = 1;
    }
}

static E_RM_ERROR rv_smc_set_steering(int i_steering, S_ROVER *ps_rover) {
    int i_idx;
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    for (i_idx = 0; !e_errstat && i_idx < I_RM_NB_SMC; i_idx++) {
        if (i_steering > 0 && ps_rover->i_steering <= 0) {  // XR: To check
            smc_set_lw_pos(E_RM_SMC_POS_RIGHT, &(ps_rover->ts_smc[i_idx]));
            smc_set_rw_pos(E_RM_SMC_POS_RIGHT, &(ps_rover->ts_smc[i_idx]));
            smc_set_lrw_dir_on_from_pos(&(ps_rover->ts_smc[i_idx]));
            e_errstat = smc_fprintf(ps_rover->ps_file, &(ps_rover->ts_smc[i_idx]));
        } else if (i_steering < 0 && ps_rover->i_steering >= 0) {
            smc_set_lw_pos(E_RM_SMC_POS_LEFT, &(ps_rover->ts_smc[i_idx]));
            smc_set_rw_pos(E_RM_SMC_POS_LEFT, &(ps_rover->ts_smc[i_idx]));
            smc_set_lrw_dir_on_from_pos(&(ps_rover->ts_smc[i_idx]));
            e_errstat = smc_fprintf(ps_rover->ps_file, &(ps_rover->ts_smc[i_idx]));
        } else if (i_steering == 0 && ps_rover->i_steering != 0) {
            smc_set_lrw_on(I_RM_FALSE, &(ps_rover->ts_smc[i_idx]));
            e_errstat = smc_fprintf(ps_rover->ps_file, &(ps_rover->ts_smc[i_idx]));
        }
    }
    ps_rover->i_steering = i_steering;
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

static E_RM_ERROR rv_smc_set_configuration(E_RM_CONFIG e_config, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    E_RM_ERROR e_errstat1;
    E_RM_ERROR e_errstat2;
    T_RM_BOOLEAN b_continue;
    int i_idx;

    printf("rv_smc_set_configuration\n");
    /* set wheels position */
    e_errstat = E_RM_ERROR_OK;
    e_errstat1 = E_RM_ERROR_OK;
    e_errstat2 = E_RM_ERROR_OK;
    ps_rover->e_config = e_config;
    switch (e_config) {
        case E_RM_CONFIG_STRAIGHT:
        case E_RM_CONFIG_CRAB:
        case E_RM_CONFIG_TANK:
        case E_RM_CONFIG_ACKERMANN:
            smc_set_lrw_pos(E_RM_SMC_POS_MID, &(ps_rover->ts_smc[E_RM_CARD_0]));
            smc_set_lrw_pos(E_RM_SMC_POS_MID, &(ps_rover->ts_smc[E_RM_CARD_1]));
            smc_set_lrw_pos(E_RM_SMC_POS_MID, &(ps_rover->ts_smc[E_RM_CARD_2]));
            break;
        case E_RM_CONFIG_CIRCLE:
            if (ps_rover->e_model != E_RM_iartemis) {
                smc_set_lw_pos(E_RM_SMC_POS_MID, &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_LEFT]));
                smc_set_rw_pos(E_RM_SMC_POS_RIGHT, &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_LEFT]));
                smc_set_lw_pos(E_RM_SMC_POS_LEFT, &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_RIGHT]));
                smc_set_rw_pos(E_RM_SMC_POS_MID, &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_RIGHT]));
                smc_set_lw_pos(E_RM_SMC_POS_LEFT, &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_REAR]));
                smc_set_rw_pos(E_RM_SMC_POS_RIGHT, &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_REAR]));
            } else {
                smc_set_lw_pos(E_RM_SMC_POS_RIGHT, &(ps_rover->ts_smc[I_RM_iartemis_CARD_FRONT]));
                smc_set_rw_pos(E_RM_SMC_POS_LEFT, &(ps_rover->ts_smc[I_RM_iartemis_CARD_FRONT]));
                smc_set_lrw_pos(E_RM_SMC_POS_MID, &(ps_rover->ts_smc[I_RM_iartemis_CARD_MIDDLE]));
                smc_set_lw_pos(E_RM_SMC_POS_LEFT, &(ps_rover->ts_smc[I_RM_iartemis_CARD_REAR]));
                smc_set_rw_pos(E_RM_SMC_POS_RIGHT, &(ps_rover->ts_smc[I_RM_iartemis_CARD_REAR]));
            }
            break;
        default:
            smc_set_lrw_pos(E_RM_SMC_POS_MID, &(ps_rover->ts_smc[E_RM_CARD_0]));
            smc_set_lrw_pos(E_RM_SMC_POS_MID, &(ps_rover->ts_smc[E_RM_CARD_1]));
            smc_set_lrw_pos(E_RM_SMC_POS_MID, &(ps_rover->ts_smc[E_RM_CARD_2]));
            ps_rover->e_config = E_RM_CONFIG_UNKNOWN;
            break;
    }
    /* active wait */
    b_continue = I_RM_TRUE;
    while (b_continue && !gb_emergencystop) {
        dcmc_rv_debug(ps_rover);
        b_continue = I_RM_FALSE;
        for (i_idx = 0; !e_errstat && i_idx < I_RM_NB_SMC; i_idx++) {
            e_errstat1 = smc_fscanf(ps_rover->ps_file, &(ps_rover->ts_smc[i_idx]));
            smc_set_lrw_dir_on_from_pos(&(ps_rover->ts_smc[i_idx]));
            e_errstat2 = smc_fprintf(ps_rover->ps_file, &(ps_rover->ts_smc[i_idx]));
            b_continue = b_continue || smc_is_lrw_on(&(ps_rover->ts_smc[i_idx]));
        }
        e_errstat = (e_errstat1) ? e_errstat1 : e_errstat2;
        usleep(I_RM_SMC_THREAD_FREQ_MS * 1000);
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/* Apply steering orders on card */
static E_RM_ERROR rv_smc_apply(S_ROVER *ps_rover) {
    int i_idx;
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    if (ps_rover->b_smc_modified) {
        for (i_idx = 0; !e_errstat && i_idx < I_RM_NB_DMC; i_idx++) {
            e_errstat = smc_fprintf(ps_rover->ps_file, &(ps_rover->ts_smc[i_idx]));
        }
        ps_rover->b_smc_modified = 0;
    }
    return e_errstat;
}

/*------------------*/
/* Thread functions */
/*------------------*/

// Wait 2 seconds and apply new speed
static void *rv_dmc_thread_ls_sleep(void *pv_arg) {
    S_ROVER *ps_rover;
    E_RM_ERROR e_errstat;

    pthread_detach(pthread_self());
    e_errstat = E_RM_ERROR_OK;
    ps_rover = (S_ROVER *)pv_arg;
    if (!e_errstat && usleep(I_RM_DMC_THREAD_PAUSE_MS * 1000) != 0) {
        e_errstat = E_RM_ERROR_THREAD;
    }
    if (!e_errstat) {
        pthread_mutex_lock(&(ps_rover->o_dmc_mutex));  // XR: global lock
        rv_dmc_set_ls_on(I_RM_TRUE, ps_rover);
        e_errstat = rv_dmc_apply(ps_rover);
        ps_rover->b_dmc_ls_sleep = I_RM_FALSE;
        pthread_mutex_unlock(&(ps_rover->o_dmc_mutex));  // XR: global lock
    }
    PRINT_ERROR(e_errstat);
    return NULL;
}

// Wait 2 seconds and apply new speed
static void *rv_dmc_thread_rs_sleep(void *pv_arg) {
    S_ROVER *ps_rover;
    E_RM_ERROR e_errstat;

    pthread_detach(pthread_self());
    e_errstat = E_RM_ERROR_OK;
    ps_rover = (S_ROVER *)pv_arg;
    if (!e_errstat && usleep(I_RM_DMC_THREAD_PAUSE_MS * 1000) != 0) {
        e_errstat = E_RM_ERROR_THREAD;
    }
    if (!e_errstat) {
        pthread_mutex_lock(&(ps_rover->o_dmc_mutex));  // XR: global lock
        rv_dmc_set_rs_on(I_RM_TRUE, ps_rover);
        e_errstat = rv_dmc_apply(ps_rover);
        ps_rover->b_dmc_rs_sleep = I_RM_FALSE;
        pthread_mutex_unlock(&(ps_rover->o_dmc_mutex));  // XR: global lock
    }
    PRINT_ERROR(e_errstat);
    return NULL;
}

// Stop wheels if needed
static void *rv_smc_thread(void *pv_arg) {
    S_ROVER *ps_rover;
    T_RM_BOOLEAN b_apply;
    E_RM_ERROR e_errstat;
    T_RM_BOOLEAN b_exit;
    int i_idx;

    b_exit = I_RM_FALSE;
    e_errstat = E_RM_ERROR_OK;
    ps_rover = (S_ROVER *)pv_arg;
    pthread_detach(pthread_self());
    while (!b_exit) {
        b_apply = I_RM_FALSE;
        pthread_mutex_lock(&(ps_rover->o_smc_mutex));  // XR: global lock
        /*printf("."); fflush(stdout);*/
        /* check steering */
        for (i_idx = 0; !e_errstat && i_idx < I_RM_NB_SMC; i_idx++) {
            e_errstat = smc_fscanf(ps_rover->ps_file, &(ps_rover->ts_smc[i_idx]));
            b_apply = smc_crab_checkstop_lrw(&(ps_rover->ts_smc[i_idx])) || b_apply;  // b_apply at right of ||
        }
        if (!e_errstat && b_apply) {
            ps_rover->b_smc_modified = I_RM_TRUE;
            e_errstat = rv_smc_apply(ps_rover);
        }
        pthread_mutex_unlock(&(ps_rover->o_smc_mutex));  // XR: global lock
        PRINT_ERROR(e_errstat);
        /* wait some time, exit if can't be done */
        if (usleep(I_RM_SMC_THREAD_FREQ_MS * 1000) != 0) {
            e_errstat = E_RM_ERROR_THREAD;
            b_exit = I_RM_TRUE;
        }
    }
    PRINT_ERROR(e_errstat);
    return NULL;
}

/*--------------------------------------*/
/* Private ROVER functions (A2TS CARDS) */
/*--------------------------------------*/
static E_RM_ERROR rv_a2ts_init(S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    E_RM_ERROR e_errstat1;
    E_RM_ERROR e_errstat2;

    /* Initialize all cards */
    rv_dmc_init(ps_rover);
    rv_dmc_set_speed_uc(0, ps_rover);
    rv_dmc_set_ls_on(I_RM_FALSE, ps_rover);
    rv_dmc_set_rs_on(I_RM_FALSE, ps_rover);
    rv_dmc_set_ls_dir(E_RM_DMC_DIR_POS, ps_rover);
    rv_dmc_set_rs_dir(E_RM_DMC_DIR_POS, ps_rover);
    rv_dmc_set_brake(E_RM_BRAKE_OFF, ps_rover);
    rv_dmc_set_gear(E_RM_GEAR_LOW, ps_rover);
    rv_smc_init(ps_rover);
    rv_smc_set_on(I_RM_FALSE, ps_rover);
    e_errstat1 = rv_dmc_apply(ps_rover);
    e_errstat2 = rv_smc_apply(ps_rover);
    e_errstat = (e_errstat1) ? e_errstat1 : e_errstat2;
    return e_errstat;
}

static E_RM_ERROR rv_a2ts_close(S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    e_errstat = E_RM_ERROR_OK;
    fclose(ps_rover->ps_file);
    return e_errstat;
}

static E_RM_ERROR rv_a2ts_set_configuration(E_RM_CONFIG e_config, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    if (!e_errstat) {
        pthread_mutex_lock(&(ps_rover->o_smc_mutex));
        if (!e_errstat && ps_rover->o_smc_thread != pthread_self() && e_config != E_RM_CONFIG_CRAB) {
            /* cancel steering thread if previous config is crab and new config is not crab */
            if (pthread_cancel(ps_rover->o_smc_thread) != 0) {
                e_errstat = E_RM_ERROR_THREAD;
            } else {
                ps_rover->o_smc_thread = pthread_self();
                printf("thread canceled\n");
            }
        }
        if (!e_errstat && ps_rover->o_smc_thread == pthread_self() && e_config == E_RM_CONFIG_CRAB) {
            /* start steering thread if previous config is not crab and new config is crab */
            if (pthread_create(&(ps_rover->o_smc_thread), NULL, rv_smc_thread, (void *)ps_rover) != 0) {
                e_errstat = E_RM_ERROR_THREAD;
            } else {
                printf("thread created\n");
            }
        }
        e_errstat = rv_smc_set_configuration(e_config, ps_rover);
        if (e_errstat) {
            // XR: emergency stop
        }
        pthread_mutex_unlock(&(ps_rover->o_smc_mutex));
    }
    return e_errstat;
}

static E_RM_ERROR rv_a2ts_set_speed_pct(int i_speed, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    if (!e_errstat) {
        /* check if a pause is needed before changing direction */
        if ((i_speed > 0 && ps_rover->e_dmc_ls_dir == E_RM_DMC_DIR_NEG) ||
            (i_speed < 0 && ps_rover->e_dmc_ls_dir == E_RM_DMC_DIR_POS) ||
            (ps_rover->e_config == E_RM_CONFIG_TANK && i_speed > 0 && ps_rover->e_dmc_rs_dir == E_RM_DMC_DIR_POS) ||
            (ps_rover->e_config == E_RM_CONFIG_TANK && i_speed < 0 && ps_rover->e_dmc_rs_dir == E_RM_DMC_DIR_NEG)) {
            /*e_errstat = dcmc_rv_set_idle(ps_rover);*/
            rv_dmc_set_on(I_RM_FALSE, ps_rover);
            rv_dmc_apply(ps_rover);
            usleep(I_RM_PAUSE_MS * 1000);
        }
    }
    if (!e_errstat) {
        /* set driving direction and speed */
        if (i_speed > 0) {
            if (ps_rover->e_config ==
                E_RM_CONFIG_CIRCLE /*|| ps_rover->e_config == E_RM_CONFIG_TANK*/) {  // XR tank pas necessaire
                rv_dmc_set_ls_dir(E_RM_DMC_DIR_POS, ps_rover);
                rv_dmc_set_rs_dir(E_RM_DMC_DIR_NEG, ps_rover);
            } else {
                rv_dmc_set_dir(E_RM_DMC_DIR_POS, ps_rover);
            }
            rv_dmc_set_on(I_RM_TRUE, ps_rover);
        } else if (i_speed < 0) {
            if (ps_rover->e_config ==
                E_RM_CONFIG_CIRCLE /*|| ps_rover->e_config == E_RM_CONFIG_TANK*/) {  // XR tank pas necessaire
                rv_dmc_set_ls_dir(E_RM_DMC_DIR_NEG, ps_rover);
                rv_dmc_set_rs_dir(E_RM_DMC_DIR_POS, ps_rover);
            } else {
                rv_dmc_set_dir(E_RM_DMC_DIR_NEG, ps_rover);
            }
            rv_dmc_set_on(I_RM_TRUE, ps_rover);
        } else {
            rv_dmc_set_on(I_RM_FALSE, ps_rover);
        }
        rv_dmc_set_speed_uc(uchar_get_from_pct(i_speed), ps_rover);
        e_errstat = rv_dmc_apply(ps_rover);  // XR: global lock
    }
    return e_errstat;
}

static E_RM_ERROR rv_a2ts_set_left_side_speed_pct(int i_speed, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    E_RM_DMC_DIR e_ls_dir;

    e_errstat = E_RM_ERROR_OK;
    if (!e_errstat) {
        e_ls_dir = ps_rover->e_dmc_ls_dir;
        if (i_speed > 0) {
            e_ls_dir = E_RM_DMC_DIR_POS;
        } else if (i_speed < 0) {
            e_ls_dir = E_RM_DMC_DIR_NEG;
        }
    }
    if (!e_errstat && !ps_rover->b_dmc_ls_sleep) {
        /*e_ls_dir = ps_rover->e_dmc_ls_dir;*/
        /*if( i_speed > 0 ){*/
        /*e_ls_dir = E_RM_DMC_DIR_FWD;*/
        /*}*/
        /*else if( i_speed < 0 ){*/
        /*e_ls_dir = E_RM_DMC_DIR_BWD;*/
        /*}*/
        /* check if a pause is needed before changing speed */
        if ((i_speed > 0 && ps_rover->e_dmc_ls_dir == E_RM_DMC_DIR_NEG) ||
            (i_speed < 0 && ps_rover->e_dmc_ls_dir == E_RM_DMC_DIR_POS)) {
            pthread_mutex_lock(&(ps_rover->o_dmc_mutex));
            rv_dmc_set_ls_on(I_RM_FALSE, ps_rover);
            e_errstat = rv_dmc_apply(ps_rover);
            /* set driving direction and speed */
            rv_dmc_set_ls_dir(e_ls_dir, ps_rover);
            rv_dmc_set_ls_speed_uc(uchar_get_from_pct(i_speed), ps_rover);
            ps_rover->b_dmc_ls_sleep = I_RM_TRUE;
            pthread_mutex_unlock(&(ps_rover->o_dmc_mutex));
            if (pthread_create(&(ps_rover->o_dmc_ls_thread), NULL, rv_dmc_thread_ls_sleep, (void *)ps_rover) != 0) {
                e_errstat = E_RM_ERROR_THREAD;
            }
        } else {
            /* set driving direction and speed */
            pthread_mutex_lock(&(ps_rover->o_dmc_mutex));  // XR: check if needed there
            rv_dmc_set_ls_dir(e_ls_dir, ps_rover);
            rv_dmc_set_ls_speed_uc(uchar_get_from_pct(i_speed), ps_rover);
            rv_dmc_set_ls_on(I_RM_TRUE, ps_rover);
            e_errstat = rv_dmc_apply(ps_rover);  // XR: global lock
            pthread_mutex_unlock(&(ps_rover->o_dmc_mutex));
        }
    } else if (!e_errstat) {
        pthread_mutex_lock(&(ps_rover->o_dmc_mutex));  // XR: check if needed there
        rv_dmc_set_ls_dir(e_ls_dir, ps_rover);
        rv_dmc_set_ls_speed_uc(uchar_get_from_pct(i_speed), ps_rover);
        pthread_mutex_unlock(&(ps_rover->o_dmc_mutex));
    }
    return e_errstat;
}

static E_RM_ERROR rv_a2ts_set_right_side_speed_pct(int i_speed, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    E_RM_DMC_DIR e_rs_dir;

    e_errstat = E_RM_ERROR_OK;
    if (!e_errstat) {
        e_rs_dir = ps_rover->e_dmc_rs_dir;
        if (i_speed > 0) {
            e_rs_dir = E_RM_DMC_DIR_POS;
        } else if (i_speed < 0) {
            e_rs_dir = E_RM_DMC_DIR_NEG;
        }
    }
    if (!e_errstat && !ps_rover->b_dmc_rs_sleep) {
        /*e_rs_dir = ps_rover->e_dmc_rs_dir;*/
        /*if( i_speed > 0 ){*/
        /*e_rs_dir = E_RM_DMC_DIR_FWD;*/
        /*}*/
        /*else if( i_speed < 0 ){*/
        /*e_rs_dir = E_RM_DMC_DIR_BWD;*/
        /*}*/
        /* check if a pause is needed before changing speed */
        if ((i_speed > 0 && ps_rover->e_dmc_rs_dir == E_RM_DMC_DIR_NEG) ||
            (i_speed < 0 && ps_rover->e_dmc_rs_dir == E_RM_DMC_DIR_POS)) {
            pthread_mutex_lock(&(ps_rover->o_dmc_mutex));
            rv_dmc_set_rs_on(I_RM_FALSE, ps_rover);
            e_errstat = rv_dmc_apply(ps_rover);
            /* set driving direction and speed */
            rv_dmc_set_rs_dir(e_rs_dir, ps_rover);
            rv_dmc_set_rs_speed_uc(uchar_get_from_pct(i_speed), ps_rover);
            ps_rover->b_dmc_rs_sleep = I_RM_TRUE;
            pthread_mutex_unlock(&(ps_rover->o_dmc_mutex));
            if (pthread_create(&(ps_rover->o_dmc_rs_thread), NULL, rv_dmc_thread_rs_sleep, (void *)ps_rover) != 0) {
                e_errstat = E_RM_ERROR_THREAD;
            }
        } else {
            /* set driving direction and speed */
            pthread_mutex_lock(&(ps_rover->o_dmc_mutex));
            rv_dmc_set_rs_dir(e_rs_dir, ps_rover);
            rv_dmc_set_rs_speed_uc(uchar_get_from_pct(i_speed), ps_rover);
            rv_dmc_set_rs_on(I_RM_TRUE, ps_rover);
            e_errstat = rv_dmc_apply(ps_rover);
            pthread_mutex_unlock(&(ps_rover->o_dmc_mutex));
        }
    } else if (!e_errstat) {
        pthread_mutex_lock(&(ps_rover->o_dmc_mutex));  // XR: check if needed there
        rv_dmc_set_rs_dir(e_rs_dir, ps_rover);
        rv_dmc_set_rs_speed_uc(uchar_get_from_pct(i_speed), ps_rover);
        pthread_mutex_unlock(&(ps_rover->o_dmc_mutex));
    }
    return e_errstat;
}

static E_RM_ERROR rv_a2ts_set_steering(int i_steering, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    e_errstat = E_RM_ERROR_OK;
    switch (ps_rover->e_config) {
        case E_RM_CONFIG_CRAB:
            pthread_mutex_lock(&(ps_rover->o_smc_mutex));  // XR: global lock
            e_errstat = rv_smc_set_steering(i_steering, ps_rover);
            pthread_mutex_unlock(&(ps_rover->o_smc_mutex));  // XR: global lock
            break;
        default:
            e_errstat = E_RM_ERROR_INVALID_CONFIG;
            break;
    }
    return e_errstat;
}

/*-------------------------------------*/
/* Private ROVER functions (BCB CARDS) */
/*-------------------------------------*/

static E_RM_ERROR rv_bcb_init(S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    FILE *tps_files[I_RM_NB_BCB];
    int ti_pos[I_RM_NB_BCB];
    int i_cpt;
    int i_status;
    char tc_buffer[256];

    e_errstat = E_RM_ERROR_OK;
    // init BCB cards and get cards positions
    for (i_cpt = 0; !e_errstat && i_cpt < I_RM_NB_BCB; i_cpt++) {
        i_status = 0;
        ti_pos[i_cpt] = -1;
        tps_files[i_cpt] = ps_rover->tps_bcb_files[i_cpt];
        e_errstat = bcb_write(ps_rover->tps_bcb_files[i_cpt], "bi\n");
        usleep(I_RM_BCB_WRITE_DELAY_US);
        e_errstat = bcb_write(ps_rover->tps_bcb_files[i_cpt], "bp ?\n");
        if (!e_errstat && ps_rover->e_model == E_RM_ARTEMIS_BCB) {
            e_errstat = bcb_read(ps_rover->tps_bcb_files[i_cpt], tc_buffer, 256);
            if (!e_errstat) {
                i_status = sscanf(tc_buffer, "%d", &(ti_pos[i_cpt]));
            }
            if (!e_errstat && i_status != 1) {
                e_errstat = E_RM_ERROR_READ;
            }
        } else if (!e_errstat && ps_rover->e_model == E_RM_STDOUT_BCB) {
            i_status = 1;
            ti_pos[i_cpt] = i_cpt;
        }
        if (!e_errstat && (ti_pos[i_cpt] < 0 || ti_pos[i_cpt] > 2)) {
            e_errstat = E_RM_ERROR_INVALID_PARAMETER;
        }
    }
    // reorder files descriptors
    for (i_cpt = 0; !e_errstat && i_cpt < I_RM_NB_BCB; i_cpt++) {
        ps_rover->tps_bcb_files[ti_pos[i_cpt]] = tps_files[i_cpt];
    }
    // check consistency, must have position 0, 1 and 2 in any order
    if (!e_errstat && (ti_pos[0] == ti_pos[1] || ti_pos[0] == ti_pos[2] || ti_pos[1] == ti_pos[2])) {
        e_errstat = E_RM_ERROR_INVALID_PARAMETER;
    }
    if (!e_errstat && ti_pos[0] + ti_pos[1] + ti_pos[2] != 3) {
        e_errstat = E_RM_ERROR_INVALID_PARAMETER;
    }
    return e_errstat;
}

static E_RM_ERROR rv_bcb_close(S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    int i_cpt;

    e_errstat = E_RM_ERROR_OK;
    for (i_cpt = 0; !e_errstat && i_cpt < I_RM_NB_BCB; i_cpt++) {
        fclose(ps_rover->tps_bcb_files[i_cpt]);
    }
    return e_errstat;
}

static E_RM_ERROR rv_bcb_set_configuration(E_RM_CONFIG e_config, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    ps_rover->e_config = e_config;
    bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], "ds * 0\n");
    bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], "ds * 0\n");
    bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], "ds * 0\n");
    usleep(I_RM_BCB_WRITE_DELAY_US);
    bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], "wa * 0\n");
    bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], "wa * 0\n");
    bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], "wa * 0\n");
    usleep(I_RM_BCB_WRITE_DELAY_US);
    switch (e_config) {
        case E_RM_CONFIG_STRAIGHT:
        case E_RM_CONFIG_TANK:
        case E_RM_CONFIG_CRAB:
        case E_RM_CONFIG_ACKERMANN:
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], "sa * 0\n");
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], "sa * 0\n");
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], "sa * 0\n");
            break;
        case E_RM_CONFIG_CIRCLE:
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], "sa 1 -45\n");
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], "sa 2 45\n");
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], "sa 5 45\n");
            usleep(I_RM_BCB_WRITE_DELAY_US);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], "sa 3 0\n");
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], "sa 4 0\n");
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], "sa 6 -45\n");
            break;
        default:
            ps_rover->e_config = E_RM_CONFIG_UNKNOWN;
            break;
    }
    /*usleep(I_RM_BCB_WRITE_DELAY_US);*/
    return e_errstat;
}

static E_RM_ERROR rv_bcb_set_ackermann(int i_speed, int i_steering, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    double d_steering_deg;
    double d_thetai_rad;  // angle internal <= input
    double d_thetae_rad;  // angle external
    double d_sef;         // speed external front <= input
    double d_sem;         // speed external middle
    double d_sim;         // speed internal middle
    double d_sif;         // speed internal front
    char tc_buffer[16];

    e_errstat = E_RM_ERROR_OK;
    bzero(tc_buffer, 16);
    switch (ps_rover->e_config) {
        case E_RM_CONFIG_ACKERMANN:
            d_steering_deg = ((double)i_steering) * I_RM_BCB_MAX_STEER_ANGLE_DEG / 255.0;  // XR: A ameliorer
            if (d_steering_deg < -I_RM_BCB_MAX_STEER_ANGLE_DEG) {
                d_steering_deg = -I_RM_BCB_MAX_STEER_ANGLE_DEG;
            }
            if (d_steering_deg > I_RM_BCB_MAX_STEER_ANGLE_DEG) {
                d_steering_deg = I_RM_BCB_MAX_STEER_ANGLE_DEG;
            }
            d_thetai_rad = fabs(d_steering_deg / 180.0 * M_PI);
            d_thetae_rad = atan((D_RM_ARTEMIS_HALF_LENGTH * tan(d_thetai_rad)) /
                                ((D_RM_ARTEMIS_WIDTH * tan(d_thetai_rad)) + D_RM_ARTEMIS_HALF_LENGTH));
            d_sef = (double)i_speed;
            d_sem = d_sef * cos(d_thetae_rad);
            d_sim = (D_RM_ARTEMIS_HALF_LENGTH * d_sem) /
                    (D_RM_ARTEMIS_HALF_LENGTH + (D_RM_ARTEMIS_WIDTH * tan(d_thetai_rad)));
            d_sif = d_sim / cos(d_thetai_rad);
            if ((int)abs(d_sim) < 10) {
                d_sef = 0.0;
                d_sem = 0.0;
                d_sim = 0.0;
                d_sif = 0.0;
            }
            if (i_steering < 0) {  // turn right
                // steering + is left and - is right for rover and the opposite for joystick
                sprintf(tc_buffer, "sa 1 %d\n", (int)(-d_thetae_rad * 180 / M_PI));
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
                sprintf(tc_buffer, "sa 2 %d\n", (int)(-d_thetai_rad * 180 / M_PI));
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
                sprintf(tc_buffer, "sa 5 %d\n", (int)(d_thetae_rad * 180 / M_PI));
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
                sprintf(tc_buffer, "sa 6 %d\n", (int)(d_thetai_rad * 180 / M_PI));
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
                // speed
                sprintf(tc_buffer, "ds 1 %d\n", (int)d_sef);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
                sprintf(tc_buffer, "ds 5 %d\n", (int)d_sef);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
                sprintf(tc_buffer, "ds 2 %d\n", (int)d_sif);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
                sprintf(tc_buffer, "ds 6 %d\n", (int)d_sif);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
                sprintf(tc_buffer, "ds 3 %d\n", (int)d_sem);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
                sprintf(tc_buffer, "ds 4 %d\n", (int)d_sim);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
            } else if (i_steering > 0) {  // turn left
                // steering + is left and - is right for rover and the opposite for joystick
                sprintf(tc_buffer, "sa 1 %d\n", (int)(d_thetai_rad * 180 / M_PI));
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
                sprintf(tc_buffer, "sa 2 %d\n", (int)(d_thetae_rad * 180 / M_PI));
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
                sprintf(tc_buffer, "sa 5 %d\n", (int)(-d_thetai_rad * 180 / M_PI));
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
                sprintf(tc_buffer, "sa 6 %d\n", (int)(-d_thetae_rad * 180 / M_PI));
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
                // speed
                sprintf(tc_buffer, "ds 1 %d\n", (int)d_sif);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
                sprintf(tc_buffer, "ds 5 %d\n", (int)d_sif);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
                sprintf(tc_buffer, "ds 2 %d\n", (int)d_sef);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
                sprintf(tc_buffer, "ds 6 %d\n", (int)d_sef);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
                sprintf(tc_buffer, "ds 3 %d\n", (int)d_sim);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
                sprintf(tc_buffer, "ds 4 %d\n", (int)d_sem);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
            } else {
                // steering + is left and - is right for rover and the opposite for joystick
                sprintf(tc_buffer, "sa * %d\n", (int)0);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
                // speed
                sprintf(tc_buffer, "ds * %d\n", (int)i_speed);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
                bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
            }
            ps_rover->i_speed = i_speed;
            ps_rover->i_steering = i_steering;
            break;
        default:
            e_errstat = E_RM_ERROR_INVALID_CONFIG;
            break;
    }
    /*usleep(I_RM_BCB_WRITE_DELAY_US);*/
    return e_errstat;
}

static E_RM_ERROR rv_bcb_set_speed_pct(int i_speed, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    int i_speedcircle;
    double d_ratio;
    char tc_buffer[16];

    e_errstat = E_RM_ERROR_OK;
    d_ratio = sqrt(1.0 + pow(2.0 * D_RM_ARTEMIS_HALF_LENGTH / D_RM_ARTEMIS_WIDTH, 2.0));
    bzero(tc_buffer, 16);
    switch (ps_rover->e_config) {
        case E_RM_CONFIG_STRAIGHT:
            /*case E_RM_CONFIG_TANK:*/  // XR: envoie des 0 de temps en temps => conflit avec left speed et right speed
        case E_RM_CONFIG_CRAB:
            sprintf(tc_buffer, "ds * %d\n", i_speed);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
            break;
        case E_RM_CONFIG_CIRCLE:
            i_speedcircle = (int)(i_speed * d_ratio);
            if (i_speedcircle > 100) {
                i_speedcircle = 100;
                i_speed = (int)(100.0 / d_ratio);
            } else if (i_speedcircle < -100) {
                i_speedcircle = -100;
                i_speed = (int)-(100.0 / d_ratio);
            }
            sprintf(tc_buffer, "ds 1 %d\n", i_speedcircle);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
            sprintf(tc_buffer, "ds 3 %d\n", i_speed);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
            sprintf(tc_buffer, "ds 2 %d\n", -i_speedcircle);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
            sprintf(tc_buffer, "ds 4 %d\n", -i_speed);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
            sprintf(tc_buffer, "ds 5 %d\n", i_speedcircle);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
            sprintf(tc_buffer, "ds 6 %d\n", -i_speedcircle);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
            break;
        case E_RM_CONFIG_ACKERMANN:
            e_errstat = rv_bcb_set_ackermann(i_speed, ps_rover->i_steering, ps_rover);
            break;
        default:
            e_errstat = E_RM_ERROR_INVALID_CONFIG;
            break;
    }
    /*usleep(I_RM_BCB_WRITE_DELAY_US);*/
    return e_errstat;
}

static E_RM_ERROR rv_bcb_set_left_side_speed_pct(int i_speed, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    char tc_buffer[16];

    e_errstat = E_RM_ERROR_OK;
    bzero(tc_buffer, 16);
    switch (ps_rover->e_config) {
        case E_RM_CONFIG_TANK:
            sprintf(tc_buffer, "ds * %d\n", i_speed);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
            sprintf(tc_buffer, "ds 5 %d\n", i_speed);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
            break;
        default:
            e_errstat = E_RM_ERROR_INVALID_CONFIG;
            break;
    }
    /*usleep(I_RM_BCB_WRITE_DELAY_US);*/
    return e_errstat;
}

static E_RM_ERROR rv_bcb_set_right_side_speed_pct(int i_speed, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    char tc_buffer[16];

    bzero(tc_buffer, 16);
    e_errstat = E_RM_ERROR_OK;
    switch (ps_rover->e_config) {
        case E_RM_CONFIG_TANK:
            sprintf(tc_buffer, "ds * %d\n", i_speed);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
            sprintf(tc_buffer, "ds 6 %d\n", i_speed);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
            break;
        default:
            e_errstat = E_RM_ERROR_INVALID_CONFIG;
            break;
    }
    /*usleep(I_RM_BCB_WRITE_DELAY_US);*/
    return e_errstat;
}

static E_RM_ERROR rv_bcb_set_steering(int i_steering, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    char tc_buffer[16];

    e_errstat = E_RM_ERROR_OK;
    bzero(tc_buffer, 16);
    switch (ps_rover->e_config) {
        case E_RM_CONFIG_CRAB:
            i_steering = i_steering * I_RM_BCB_MAX_STEER_ANGLE_DEG / 255;  // XR: A ameliorer
            if (i_steering < -I_RM_BCB_MAX_STEER_ANGLE_DEG) {
                i_steering = -I_RM_BCB_MAX_STEER_ANGLE_DEG;
            }
            if (i_steering > I_RM_BCB_MAX_STEER_ANGLE_DEG) {
                i_steering = I_RM_BCB_MAX_STEER_ANGLE_DEG;
            }
            sprintf(tc_buffer, "sa * %d\n",
                    -i_steering);  // + is left and - is right for rover and the opposite for joystick
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_LEFT], tc_buffer);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_RIGHT], tc_buffer);
            bcb_write(ps_rover->tps_bcb_files[I_RM_ARTEMIS_CARD_REAR], tc_buffer);
            break;
        case E_RM_CONFIG_ACKERMANN:
            e_errstat = rv_bcb_set_ackermann(ps_rover->i_speed, -i_steering, ps_rover);
            break;
        default:
            e_errstat = E_RM_ERROR_INVALID_CONFIG;
            break;
    }
    /*usleep(I_RM_BCB_WRITE_DELAY_US);*/
    return e_errstat;
}

/*--------------------*/
/* Handler for SIGINT */
/*--------------------*/
static void dcmc_rv_handler(int i_signum) {
    i_signum = i_signum;
    gb_emergencystop = I_RM_TRUE;
    dcmc_rv_emergency_stop(gps_rover);
    exit(0);
}
/* DOXYGEN : END OF STATIC */
/** @} */
/** @} */

/*-------------------------*/
/* Public ROVER functions */
/*-------------------------*/

/**
 * @addtogroup dcmc_public
 * @{
 * @name Rover motor controller API
 * @{
 */

/*!
 * @brief          Initialise rover internal structure
 * @param[in]      pc_device : device(s) to open
 * @param[in]      e_model : rover model
 * @param[in,out]  ps_rover : the rover
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_init(char *pc_device, E_RM_MODEL e_model, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    FILE *po_file;
    char *pc_token[I_RM_NB_BCB];
    struct sigaction s_action;
    int i;

    e_errstat = E_RM_ERROR_OK;
    if (pc_device == NULL || ps_rover == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (!e_errstat) {
        ps_rover->e_model = e_model;
        ps_rover->e_config = E_RM_CONFIG_UNKNOWN;
        ps_rover->b_dmc_ls_sleep = I_RM_FALSE;
        ps_rover->b_dmc_rs_sleep = I_RM_FALSE;
        ps_rover->o_dmc_ls_thread = pthread_self();
        ps_rover->o_dmc_rs_thread = pthread_self();
        ps_rover->i_speed = 0;
        ps_rover->i_steering = 0;
        ps_rover->o_smc_thread = pthread_self();
    }
    if (!e_errstat && (e_model == E_RM_ARTEMIS_BCB || e_model == E_RM_STDOUT_BCB)) {  // ARTEMIS BCB CARD
        ps_rover->e_card = E_RM_CARD_BCB;
        ps_rover->e_brake = E_RM_BRAKE_OFF;
        pc_token[0] = strtok(pc_device, ":");
        pc_token[1] = strtok(NULL, ":");
        pc_token[2] = strtok(NULL, ":");
        for (i = 0; i < I_RM_NB_BCB; i++) {
            po_file = fopen(pc_token[i], "a+");
            if (po_file == NULL) {
                perror("fopen");
                e_errstat = E_RM_ERROR_INVALID_PARAMETER;
            } else {
                ps_rover->tps_bcb_files[i] = po_file;
                setvbuf(po_file, NULL, _IONBF, 0);
                fcntl(fileno(po_file), F_SETFL, fcntl(fileno(po_file), F_GETFL) | O_DSYNC | O_RSYNC);
            }
        }
        if (!e_errstat) {
            sleep(I_RM_BCB_INIT_DELAY_S);  // wait init of BCB card
            e_errstat = rv_bcb_init(ps_rover);
        }
    } else if (!e_errstat) {  // ! BCB CARD
        ps_rover->e_card = E_RM_CARD_A2TS;
        ps_rover->ps_file = fopen(pc_device, "a+");
        if (ps_rover->ps_file == NULL) {
            perror("fopen");
            e_errstat = E_RM_ERROR_INVALID_PARAMETER;
        }
        if (!e_errstat && pthread_mutex_init(&(ps_rover->o_dmc_mutex), NULL) != 0) {
            e_errstat = E_RM_ERROR_MUTEX;
        }
        if (!e_errstat && pthread_mutex_init(&(ps_rover->o_smc_mutex), NULL) != 0) {
            e_errstat = E_RM_ERROR_MUTEX;
        }
        if (!e_errstat) {
            e_errstat = rv_a2ts_init(ps_rover);
        }
    }
    if (!e_errstat) {
        gb_emergencystop = I_RM_FALSE;
        gps_rover = ps_rover;
        s_action.sa_handler = dcmc_rv_handler;
        sigemptyset(&s_action.sa_mask);
        sigaction(SIGINT, &s_action, NULL);
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/*!
 * @brief          Close rover internal structure
 * @param[in]               :
 * @param[in]               :
 * @param[in,out]  ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_close(S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    if (ps_rover == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (!e_errstat) {
        /* Stop everything  */
        e_errstat = dcmc_rv_emergency_stop(ps_rover);
        /* Close opened files */
        if (ps_rover->e_card == E_RM_CARD_BCB) {
            e_errstat = rv_bcb_close(ps_rover);
        } else if (ps_rover->e_card == E_RM_CARD_A2TS) {
            e_errstat = rv_a2ts_close(ps_rover);
        }
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/*!
 * @brief          Stop rover
 * @param[in]               :
 * @param[in]               :
 * @param[in,out]  ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_emergency_stop(S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    E_RM_ERROR e_errstat1;
    E_RM_ERROR e_errstat2;

    e_errstat = E_RM_ERROR_OK;
    if (ps_rover == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (!e_errstat) {
        /* Kill threads */
        if (ps_rover->o_smc_thread != pthread_self()) {
            pthread_cancel(ps_rover->o_smc_thread);
            ps_rover->o_smc_thread = pthread_self();
        }
        if (ps_rover->o_dmc_ls_thread != pthread_self()) {
            pthread_cancel(ps_rover->o_dmc_ls_thread);
            ps_rover->o_dmc_ls_thread = pthread_self();
        }
        if (ps_rover->o_dmc_rs_thread != pthread_self()) {
            pthread_cancel(ps_rover->o_dmc_rs_thread);
            ps_rover->o_dmc_rs_thread = pthread_self();
        }
        /* Stop rover */
        rv_dmc_set_speed_uc(0, ps_rover);
        rv_dmc_set_ls_on(I_RM_FALSE, ps_rover);
        rv_dmc_set_rs_on(I_RM_FALSE, ps_rover);
        rv_smc_set_on(I_RM_FALSE, ps_rover);
        e_errstat1 = rv_dmc_apply(ps_rover);
        e_errstat2 = rv_smc_apply(ps_rover);
        e_errstat = (e_errstat1) ? e_errstat1 : e_errstat2;
        /* Unset configuration */
        ps_rover->e_config = E_RM_CONFIG_UNKNOWN;
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/*!
 * @brief          Print rover state on stdout
 * @param[in]      ps_rover :
 * @return
 */
void dcmc_rv_debug(S_ROVER *ps_rover) {
    if (ps_rover->e_card == E_RM_CARD_A2TS) {
        if (ps_rover->e_model == E_RM_iartemis || ps_rover->e_model == E_RM_STDOUT) {
            printf("iartemis\n");
            dmc_fscanf(ps_rover->ps_file, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
            smc_debug(stdout, &(ps_rover->ts_smc[I_RM_iartemis_CARD_FRONT]));
            dmc_debug(stdout, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
            printf("\n");
            smc_debug(stdout, &(ps_rover->ts_smc[I_RM_iartemis_CARD_MIDDLE]));
            dmc_debug(stdout, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
            printf("\n");
            smc_debug(stdout, &(ps_rover->ts_smc[I_RM_iartemis_CARD_REAR]));
            dmc_debug(stdout, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));
            printf("\n");
        } else {  // ARTEMIS and ...
            printf("ARTEMIS\n");
            smc_right_debug(stdout, &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_LEFT]));
            smc_left_debug(stdout, &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_RIGHT]));
            dmc_right_debug(stdout, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_LEFT]));
            dmc_left_debug(stdout, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_RIGHT]));
            printf("\n");
            smc_left_debug(stdout, &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_LEFT]));
            smc_right_debug(stdout, &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_RIGHT]));
            dmc_left_debug(stdout, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_LEFT]));
            dmc_right_debug(stdout, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_RIGHT]));
            printf("\n");
            smc_debug(stdout, &(ps_rover->ts_smc[I_RM_ARTEMIS_CARD_REAR]));
            dmc_debug(stdout, &(ps_rover->ts_dmc[I_RM_ARTEMIS_CARD_REAR]));
            printf("\n");
        }
    }
}

/*!
 * @brief          Stop all motors of rover
 * @param[in]      ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_set_idle(S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    E_RM_ERROR e_errstat1;
    E_RM_ERROR e_errstat2;

    e_errstat = E_RM_ERROR_OK;
    if (ps_rover == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (!e_errstat) {
        if (ps_rover->e_card == E_RM_CARD_A2TS) {
            rv_dmc_set_on(I_RM_FALSE, ps_rover);
            rv_smc_set_on(I_RM_FALSE, ps_rover);
            e_errstat1 = rv_dmc_apply(ps_rover);
            e_errstat2 = rv_smc_apply(ps_rover);
            e_errstat = (e_errstat1) ? e_errstat1 : e_errstat2;
        } else if (ps_rover->e_card == E_RM_CARD_BCB) {
            rv_bcb_set_speed_pct(0, ps_rover);
            rv_bcb_set_steering(0, ps_rover);
        }
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/*!
 * @brief          Return true if rover is idle
 * @param[in]      ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
static T_RM_BOOLEAN dcmc_rv_is_idle(S_ROVER *ps_rover) {
    T_RM_BOOLEAN b_idle;

    b_idle = I_RM_TRUE;
    if (ps_rover->e_card == E_RM_CARD_A2TS) {
        if (ps_rover->b_dmc_modified || ps_rover->b_smc_modified) {
            /* can't say if driving or steering motor are idle because something has changed */
            b_idle = I_RM_FALSE;
        } else if ((ps_rover->b_dmc_ls_on && ps_rover->uc_dmc_ls_speed != 0) ||
                   (ps_rover->b_dmc_rs_on && ps_rover->uc_dmc_rs_speed != 0) || ps_rover->b_smc_on ||
                   ps_rover->b_dmc_ls_sleep || ps_rover->b_dmc_rs_sleep) {
            b_idle = I_RM_FALSE;
        }
    }
    return b_idle;
}

/*!
 * @brief          Set ou unset brake of rover
 * @param[in]      e_brake  :
 * @param[in,out]  ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_set_brake(E_RM_BRAKE e_brake, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;
    e_errstat = E_RM_ERROR_OK;
    if (ps_rover == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (ps_rover->e_model == E_RM_iartemis) {
        if (!e_errstat && !dcmc_rv_is_idle(ps_rover)) {
            e_errstat = E_RM_ERROR_INVALID_STATE;
        }
        if (!e_errstat) {
            rv_dmc_set_brake(e_brake, ps_rover);
            e_errstat = rv_dmc_apply(ps_rover);
        }
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/*!
 * @brief          Set ou unset brake of rover
 * @param[in,out]  ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_toggle_brake(S_ROVER *ps_rover) {
    E_RM_BRAKE e_brake_opp;
    if (ps_rover->e_brake == E_RM_BRAKE_ON) {
        e_brake_opp = E_RM_BRAKE_OFF;
    } else {
        e_brake_opp = E_RM_BRAKE_ON;
    }
    return dcmc_rv_set_brake(e_brake_opp, ps_rover);
}

/*!
 * @brief          Set gear of rover
 * @param[in]      e_gear   :
 * @param[in,out]  ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_set_gear(E_RM_GEAR e_gear, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    e_gear = e_gear;
    if (ps_rover == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (ps_rover->e_model == E_RM_iartemis) {
        if (!e_errstat && !dcmc_rv_is_idle(ps_rover)) {
            e_errstat = E_RM_ERROR_INVALID_STATE;
        }
        if (!e_errstat) {
            /* Feature Redmine #41 */
            /*rv_dmc_set_gear(e_gear, ps_rover);*/
            /*e_errstat = rv_dmc_apply(ps_rover);*/
        }
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/*!
 * @brief          Set gear of rover
 * @param[in,out]  ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_toggle_gear(S_ROVER *ps_rover) {
    return dcmc_rv_set_gear(ps_rover->e_gear, ps_rover);
    // ORIGINAL FILE: return dcmc_rv_set_gear(!ps_rover->e_gear, ps_rover);
}

/*!
 * @brief          Set speed of rover
 * @param[in]      i_speed  :
 * @param[in,out]  ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_set_speed_pct(int i_speed, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    if (ps_rover == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (!e_errstat && ps_rover->e_config == E_RM_CONFIG_UNKNOWN) {
        e_errstat = E_RM_ERROR_INVALID_CONFIG;
    }
    if (!e_errstat && ps_rover->e_brake == E_RM_BRAKE_ON) {
        e_errstat = E_RM_ERROR_INVALID_STATE;
    }
    if (!e_errstat) {
        if (ps_rover->e_card == E_RM_CARD_BCB) {
            e_errstat = rv_bcb_set_speed_pct(i_speed, ps_rover);
        } else if (ps_rover->e_card == E_RM_CARD_A2TS) {
            e_errstat = rv_a2ts_set_speed_pct(i_speed, ps_rover);
        }
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/*!
 * @brief          Set left side speed of rover in tank configuration
 * @param[in]      i_speed  :
 * @param[in,out]  ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_set_left_side_speed_pct(int i_speed, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    if (ps_rover == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (!e_errstat && ps_rover->e_config != E_RM_CONFIG_TANK) {
        e_errstat = E_RM_ERROR_INVALID_CONFIG;
    }
    if (!e_errstat && ps_rover->e_brake == E_RM_BRAKE_ON) {
        e_errstat = E_RM_ERROR_INVALID_STATE;
    }
    if (!e_errstat) {
        if (ps_rover->e_card == E_RM_CARD_BCB) {
            e_errstat = rv_bcb_set_left_side_speed_pct(i_speed, ps_rover);
        } else if (ps_rover->e_card == E_RM_CARD_A2TS) {
            e_errstat = rv_a2ts_set_left_side_speed_pct(i_speed, ps_rover);
        }
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/*!
 * @brief          Set right side speed of rover in tank config
 * @param[in]      i_speed  :
 * @param[in,out]  ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_set_right_side_speed_pct(int i_speed, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    if (ps_rover == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (!e_errstat && ps_rover->e_config != E_RM_CONFIG_TANK) {
        e_errstat = E_RM_ERROR_INVALID_CONFIG;
    }
    if (!e_errstat && ps_rover->e_brake == E_RM_BRAKE_ON) {
        e_errstat = E_RM_ERROR_INVALID_STATE;
    }
    if (!e_errstat) {
        if (ps_rover->e_card == E_RM_CARD_BCB) {
            e_errstat = rv_bcb_set_right_side_speed_pct(i_speed, ps_rover);
        } else if (ps_rover->e_card == E_RM_CARD_A2TS) {
            e_errstat = rv_a2ts_set_right_side_speed_pct(i_speed, ps_rover);
        }
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/*!
 * @brief          Set steering of rover
 * @param[in]      i_steering : < 0 left, 0 = stop, > 0 = right
 * @param[in,out]  ps_rover   :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_set_steering(int i_steering, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    if (ps_rover == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (!e_errstat) {
        if (ps_rover->e_card == E_RM_CARD_BCB && ps_rover->e_config != E_RM_CONFIG_CRAB &&
            ps_rover->e_config != E_RM_CONFIG_ACKERMANN) {
            e_errstat = E_RM_ERROR_INVALID_CONFIG;
        } else if (ps_rover->e_card == E_RM_CARD_A2TS && ps_rover->e_config != E_RM_CONFIG_CRAB) {
            e_errstat = E_RM_ERROR_INVALID_CONFIG;
        }
    }
    if (!e_errstat) {
        if (ps_rover->e_card == E_RM_CARD_BCB) {
            e_errstat = rv_bcb_set_steering(i_steering, ps_rover);
        } else if (ps_rover->e_card == E_RM_CARD_A2TS) {
            e_errstat = rv_a2ts_set_steering(i_steering, ps_rover);
        }
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/*!
 * @brief          Set configuration of rover
 * @param[in]      e_config :
 * @param[in,out]  ps_rover :
 * @return         code of the first error encountered, otherwise OK
 */
E_RM_ERROR dcmc_rv_set_configuration(E_RM_CONFIG e_config, S_ROVER *ps_rover) {
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    if (ps_rover == NULL) {
        e_errstat = E_RM_ERROR_NULL_PARAMETER;
    }
    if (!e_errstat) {
        if (ps_rover->e_card == E_RM_CARD_BCB) {
            e_errstat = rv_bcb_set_configuration(e_config, ps_rover);
        } else if (ps_rover->e_card == E_RM_CARD_A2TS) {
            e_errstat = dcmc_rv_set_idle(ps_rover);
            /*if( !dcmc_rv_is_idle(ps_rover) ){
                e_errstat = E_RM_ERROR_INVALID_STATE;
            }*/
            if (!e_errstat) {
                e_errstat = rv_a2ts_set_configuration(e_config, ps_rover);
            }
        }
    }
    PRINT_ERROR(e_errstat);
    return e_errstat;
}

/*!
 * @brief          Set configuration of rover
 * @param[in]      ps_file : file to print error
 * @param[in]      e_errstat : error
 */
const char *dcmc_rv_errorstring(E_RM_ERROR e_errstat) {
    return tpc_error[e_errstat];
}

/* DOXYGEN : END OF EXPORTED */
/** @} */
/** @} */
