/*****************************************************************************/
/*!
 * @file     ./dcmc_rv_motor_head.h
 * @version  $Rev: 4310 $
 * @date     Creation    : 05/05/2011
 * @date     Last commit : $Date: 2013-12-03 15:58:12 +0100 (Tue, 03 Dec 2013) $
 * @date     Modified    :
 * @author   Xavier RAVE (c)
 * @project  EDRES (Environ. Develop. Robotique Exploration Spatiale)
 * @system   DRVPR
 * @brief    Rover Motor Control
 ***************************************************************************/
#if !defined(DCMC_RV_MOTOR_HEAD)
#define DCMC_RV_MOTOR_HEAD

#include "dcmc_rv_motor_typ.h"

#ifdef __cplusplus
extern "C" {  // C code export in C++
#endif

extern E_RM_ERROR dcmc_rv_init(char *pc_device, E_RM_MODEL e_model, S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_close(S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_emergency_stop(S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_set_configuration(E_RM_CONFIG e_config, S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_set_idle(S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_set_speed_pct(int i_speed, S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_set_left_side_speed_pct(int i_speed, S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_set_right_side_speed_pct(int i_speed, S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_set_gear(E_RM_GEAR e_gear, S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_set_brake(E_RM_BRAKE e_brake, S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_set_steering(int i_steering, S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_toggle_gear(S_ROVER *ps_rover);
extern E_RM_ERROR dcmc_rv_toggle_brake(S_ROVER *ps_rover);
extern void dcmc_rv_debug(S_ROVER *ps_rover);
extern const char *dcmc_rv_errorstring(E_RM_ERROR e_errstat);

#ifdef __cplusplus
}  // End of C export
#endif

#endif
