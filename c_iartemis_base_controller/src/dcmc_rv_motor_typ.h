/*****************************************************************************/
/*!
 * @file     ./dcmc_rv_motor_typ.h
 * @version  $Rev: 4703 $
 * @date     Creation    : 05/04/2011
 * @date     Last commit : $Date: 2015-01-13 10:56:11 +0100 (Tue, 13 Jan 2015) $
 * @date     Modified    :
 * @author   Xavier RAVE (c)
 * @project  EDRES (Environ. Develop. Robotique Exploration Spatiale)
 * @system   DRVPR
 * @brief    Rover Motor Control
 ***************************************************************************/
#if !defined(DCMC_RV_MOTOR_TYP)
#define DCMC_RV_MOTOR_TYP

#include <stdio.h>
#include <pthread.h>

/*! @addtogroup dcmc_typ
 * @{
 */

/*---------------*/
/* General types */
/*---------------*/

/* BOOLEAN */
typedef int T_RM_BOOLEAN;
#define I_RM_FALSE 0
#define I_RM_TRUE 1

/* GEAR */
typedef enum { E_RM_GEAR_LOW = 0, E_RM_GEAR_HIGH = 1 } E_RM_GEAR;

/* BRAKE */
typedef enum { E_RM_BRAKE_ON = 0, E_RM_BRAKE_OFF = 1 } E_RM_BRAKE;

/* CARD TYPE */
typedef enum {                     /*  iartemis       ARTEMIS */
               E_RM_CARD_A2TS = 0, /*    X            X    */
               E_RM_CARD_BCB = 1   /*                 X    */
} E_RM_CARD_TYPE;

/* CARD POSITION */
typedef enum {                  /*  iartemis       ARTEMIS */
               E_RM_CARD_0 = 0, /*  FRONT       LEFT    */
               E_RM_CARD_1 = 1, /*  MIDDLE      RIGHT   */
               E_RM_CARD_2 = 2  /*  REAR        REAR    */
} E_RM_CARD_POS;

#define I_RM_iartemis_CARD_FRONT E_RM_CARD_0
#define I_RM_iartemis_CARD_MIDDLE E_RM_CARD_1
#define I_RM_iartemis_CARD_REAR E_RM_CARD_2

#define I_RM_ARTEMIS_CARD_LEFT E_RM_CARD_0
#define I_RM_ARTEMIS_CARD_RIGHT E_RM_CARD_1
#define I_RM_ARTEMIS_CARD_REAR E_RM_CARD_2

/*--------------------------*/
/* Driving Motor Card (DMC) */
/*--------------------------*/

#define I_RM_NB_DMC 3

typedef enum {
    E_RM_DMC_DIR_POS = 0,  // left wheel goes forward, right wheel goes backward // ~ screw
    E_RM_DMC_DIR_NEG = 1   // left wheel goes backward, right wheel goes forward // ~ unscrew
} E_RM_DMC_DIR;

typedef struct {
    unsigned char uc_comen1;
    unsigned char uc_comen2;
    unsigned char uc_razen;
    unsigned char uc_vte1;
    unsigned char uc_vte2;
} S_RM_DMC_WR_BUFF;

typedef struct {
    unsigned char uc_seuilen;
    unsigned char uc_ntope13;  // left motor nb tops on 3 bytes
    unsigned char uc_ntope12;
    unsigned char uc_ntope11;
    unsigned char uc_ntope23;  // right motor nb tops on 3 bytes
    unsigned char uc_ntope22;
    unsigned char uc_ntope21;
    unsigned char uc_perie12;  // left motor period on 2 bytes
    unsigned char uc_perie11;
    unsigned char uc_perie22;  // right motor period on 2 bytes
    unsigned char uc_perie21;
} S_RM_DMC_RD_BUFF;

typedef struct {
    char c_ident;
    S_RM_DMC_WR_BUFF s_wr_buff;
    S_RM_DMC_RD_BUFF s_rd_buff;
} S_RM_DMC;

/*---------------------------*/
/* Steering Motor Card (SMC) */
/*---------------------------*/

#define I_RM_NB_SMC 3

typedef enum {
    E_RM_SMC_DIR_RIGHT = 0, /* wheel turns right */
    E_RM_SMC_DIR_LEFT = 1   /* wheel turns left */
} E_RM_SMC_DIR;

typedef enum {
    E_RM_SMC_POS_LEFT, /* wheel if in left position */
    E_RM_SMC_POS_MID,  /* wheel if in middle position */
    E_RM_SMC_POS_RIGHT /* wheel if in right position */
} E_RM_SMC_POS;

//#define UC_RM_SMC_POS_MARGIN ((unsigned char) 2)
// Nouvelles marges pour iartemis (marges autour de la butee software)
#define UC_RM_SMC_POS_MARGIN ((unsigned char)4)

//#define UC_RM_SMC_ENDSTOP_MARGIN ((unsigned char) 4) /* */
// Nouvelles marges software pour iartemis (ces marges sont ajoutees/enlevees aux butees hardware)
#define UC_RM_SMC_ENDSTOP_MARGIN ((unsigned char)15) /* */

typedef struct {
    unsigned char uc_comax;
    unsigned char uc_razax;
    unsigned char uc_slogi;
} S_RM_SMC_WR_BUFF;

typedef struct {
    unsigned char uc_seuilax;
    unsigned char uc_elogi1;
    unsigned char uc_aglp;
    unsigned char uc_agld1;
    unsigned char uc_agld2;
    unsigned char uc_agll;
    unsigned char uc_aglm;
    unsigned char uc_agln;
    unsigned char uc_cour1;
    unsigned char uc_cour2;
} S_RM_SMC_RD_BUFF;

typedef struct {
    T_RM_BOOLEAN b_lw_on;
    T_RM_BOOLEAN b_rw_on;
    E_RM_SMC_DIR e_lw_dir;
    E_RM_SMC_DIR e_rw_dir;
    E_RM_SMC_DIR e_mintomax_dir;
    unsigned char uc_lw_agl_min;
    unsigned char uc_lw_agl_mid;
    unsigned char uc_lw_agl_max;
    unsigned char uc_lw_agl_ord;
    unsigned char uc_rw_agl_min;
    unsigned char uc_rw_agl_mid;
    unsigned char uc_rw_agl_max;
    unsigned char uc_rw_agl_ord;
} S_RM_SMC_PARAMS;

typedef struct {
    char c_ident;
    T_RM_BOOLEAN b_read; /* read device or not */
    T_RM_BOOLEAN b_simu; /* simulate move      */
    S_RM_SMC_PARAMS s_params;
    S_RM_SMC_WR_BUFF s_wr_buff;
    S_RM_SMC_RD_BUFF s_rd_buff;
} S_RM_SMC;

/*---------------------------*/
/* Bogie Control Board (BCB) */
/*---------------------------*/
#define I_RM_NB_BCB 3
#define I_RM_BCB_INIT_DELAY_S 1         /* s     */
#define I_RM_BCB_SELECT_DELAY_S 1       /* s     */
#define I_RM_BCB_WRITE_DELAY_US 0       /* us    */
#define I_RM_BCB_MAX_STEER_ANGLE_DEG 45 /* degre */

/*--------------------*/
/* Artemis dimensions */
/*--------------------*/
#define D_RM_ARTEMIS_WIDTH 1.0
#define D_RM_ARTEMIS_HALF_LENGTH 0.55

/*-------------*/
/* ROVER Types */
/*-------------*/

#define I_RM_PAUSE_MS 500 /* ms */ /* Vu avec Jean le 15/06/2015 */
//#define I_RM_SMC_THREAD_FREQ_MS 100   /* ms */
#define I_RM_SMC_THREAD_FREQ_MS 20   /* ms */
#define I_RM_DMC_THREAD_PAUSE_MS 500 /* ms */

typedef enum { E_RM_iartemis, E_RM_ARTEMIS, E_RM_ARTEMIS_BCB, E_RM_TEST, E_RM_STDOUT, E_RM_STDOUT_BCB } E_RM_MODEL;

typedef enum {
    E_RM_ERROR_OK = 0,
    E_RM_ERROR_NULL_PARAMETER,
    E_RM_ERROR_INVALID_PARAMETER,
    E_RM_ERROR_INVALID_CONFIG,
    E_RM_ERROR_INVALID_STATE,
    E_RM_ERROR_READ,
    E_RM_ERROR_WRITE,
    E_RM_ERROR_THREAD,
    E_RM_ERROR_MUTEX,
    E_RM_ERROR_TIMEOUT
} E_RM_ERROR;

typedef enum {
    E_RM_CONFIG_STRAIGHT,
    E_RM_CONFIG_CRAB,
    E_RM_CONFIG_CIRCLE,
    E_RM_CONFIG_TANK,
    E_RM_CONFIG_ACKERMANN,
    E_RM_CONFIG_UNKNOWN
} E_RM_CONFIG;

typedef struct {
    FILE *ps_file;                    /* A2TS interface file */
    FILE *tps_bcb_files[I_RM_NB_BCB]; /* BCB interface files */
    E_RM_MODEL e_model;
    E_RM_CARD_TYPE e_card;
    E_RM_CONFIG e_config;
    E_RM_BRAKE e_brake;
    E_RM_GEAR e_gear;
    /* driving data */
    T_RM_BOOLEAN b_dmc_ls_on;
    T_RM_BOOLEAN b_dmc_rs_on;
    E_RM_DMC_DIR e_dmc_ls_dir;
    E_RM_DMC_DIR e_dmc_rs_dir;
    int i_speed; /* for ackermann configuration */
    unsigned char uc_dmc_ls_speed;
    unsigned char uc_dmc_rs_speed;
    T_RM_BOOLEAN b_dmc_ls_sleep;
    T_RM_BOOLEAN b_dmc_rs_sleep;
    int b_dmc_modified; /* set to 1 if something has to be applied */
    S_RM_DMC ts_dmc[I_RM_NB_DMC];
    /* driving thread */
    pthread_t o_dmc_ls_thread;   /* o = opaque */
    pthread_t o_dmc_rs_thread;   /* o = opaque */
    pthread_mutex_t o_dmc_mutex; /* o = opaque */
    /* steering data */
    int b_smc_on;
    int i_steering;
    int b_smc_modified; /* set to 1 if something has to be applied */
    S_RM_SMC ts_smc[I_RM_NB_SMC];
    /* steering thread */
    pthread_t o_smc_thread;      /* o = opaque */
    pthread_mutex_t o_smc_mutex; /* o = opaque */
} S_ROVER;

/*! @} */

#endif
