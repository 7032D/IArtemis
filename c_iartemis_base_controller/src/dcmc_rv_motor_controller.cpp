/*****************************************************************************/
/*!
 * @file     ./dcmc_rv_motor_controller.cpp
 * @version  1.0
 * @date     Creation    : 24/03/2016
 * @date     Modified    : 29/04/2016
 * @author   Florian MARTIN (c)
 * @project  iartemis ROS
 * @system   DRVPR
 * @brief    Communication between motors box and ros node
 * @note     Watch of signs changes
 *****************************************************************************/
#include "ros/ros.h"
#include "dcmc_rv_motor.c"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "c_iartemis_base_controller/vfw_int32.h"
#include "c_iartemis_base_controller/vfw_float32.h"
#include "std_msgs/String.h"
#include <cmath>
#include <sstream>

#define I_NB_WHEEL 6

typedef struct {
    FILE *ps_file;
    E_RM_DMC_DIR e_dmc_ls_dir;    // left side direction 
    E_RM_DMC_DIR e_dmc_rs_dir;    // right side direction
    T_RM_BOOLEAN b_dmc_ls_on;     // left side on/off
    T_RM_BOOLEAN b_dmc_rs_on;     // right side on/off
    E_RM_BRAKE e_brake;           // brake on/off (when start need to be set off)
    E_RM_GEAR e_gear;             // gear (only low gear)
    unsigned char uc_dmc_ls_speed;// left side speed
    unsigned char uc_dmc_rs_speed;// right side speed
    int b_dmc_modified;           // set to 1 if something has to be applied
    S_RM_DMC ts_dmc[I_RM_NB_DMC]; //wheeling write and read buffer
    S_RM_DMC ts_smc[I_RM_NB_SMC]; //steering write and read buffer 
} S_iartemis;                        

class DcmcRvMotorController {
  public:
    DcmcRvMotorController();
    //~DcmcRvMotorController();
    void set_stop_state();

  private:
    void rv_dmc_init_iartemis(S_iartemis *ps_rover);
    E_RM_ERROR rv_dmc_apply_iartemis(S_iartemis *ps_rover);
    void rv_dmc_set_ls_dir_iartemis(E_RM_DMC_DIR e_direction, S_iartemis *ps_rover);
    void rv_dmc_set_rs_dir_iartemis(E_RM_DMC_DIR e_direction, S_iartemis *ps_rover);
    void rv_dmc_set_ls_speed_iartemis(const c_iartemis_base_controller::vfw_float32 vf_rpm_set_l, S_iartemis *ps_rover);
    void rv_dmc_set_rs_speed_iartemis(const c_iartemis_base_controller::vfw_float32 vf_rpm_set_r, S_iartemis *ps_rover);
    void rv_dmc_set_ls_on_iartemis(T_RM_BOOLEAN e_boolean, S_iartemis *ps_rover);
    void rv_dmc_set_rs_on_iartemis(T_RM_BOOLEAN e_boolean, S_iartemis *ps_rover);
    void rv_dmc_set_brake_iartemis(E_RM_BRAKE e_brake, S_iartemis *ps_rover);
    void rv_dmc_set_gear_iartemis(E_RM_GEAR e_gear, S_iartemis *ps_rover);
    void rv_dmc_set_abs_speed(S_iartemis *ps_rover);
    void rpm_callback(const c_iartemis_base_controller::vfw_float32 vf_rpm_cb);
    void generateOdomMsg(int i_diff_top_rw, int i_diff_top_lw);
    void controllerTimerCallback(const ros::TimerEvent &e);
    float radsFromEncoderCount(int i_count, double time);
    float rpmFromEncoderCount(int i_count, double time);
    float rpmFromPerie(int i_perie);
    float radsFromPerie(int i_perie);
    void updateParameters();

    
    /* ROS */
    ros::Publisher odom_pub;
    ros::Publisher rpm_pub;
    ros::Publisher top_pub;
    ros::Publisher perie_pub;
    ros::Subscriber rpm_sub;
    ros::NodeHandle nh, _nh;
    ros::Timer controllerTimer_dcmc;
    ros::Time t_current_time;
    ros::Time t_last_time;
    ros::Time t_lastRPMMessageTime;

    /* Rover dedicated structures */
    S_iartemis s_rover;
    S_iartemis *ps_iartemis;
    E_RM_ERROR e_errstat;
    E_RM_DMC_DIR e_ls_dir;                                         // Right side direction (beware of change delay with pid_node)
    E_RM_DMC_DIR e_rs_dir;                                         // Right side direction
    c_iartemis_base_controller::vfw_float32 vfw_null;              // Null vector
    c_iartemis_base_controller::vfw_float32 vf_rpm_vel;            // Velocity send to motors (before scaling to 0-255)
    c_iartemis_base_controller::vfw_float32 vf_rpm_vel_abs;
    c_iartemis_base_controller::vfw_float32 rpm_msg;               // Rpm value publish to pid_node
    float tf_rpm_msg_[I_NB_WHEEL] = {0, 0, 0, 0, 0, 0};            // Rpm value before set into rpm_msg_
    int vi_nb_top_diff_[I_NB_WHEEL] = {0, 0, 0, 0, 0, 0};          // Nb of encoder tic between two timer callback
    int ti_lastEncMessage_[I_NB_WHEEL] = {0, 0, 0, 0, 0, 0};       // Last encoder value
    int ti_last2EncMessage_[I_NB_WHEEL] = {0, 0, 0, 0, 0, 0};      // We save last 3 encoder values to distingue 0 values and low values
    int ti_last3EncMessage_[I_NB_WHEEL] = {0, 0, 0, 0, 0, 0};      
    int ti_lastPerieMessage_[I_NB_WHEEL] = {0, 0, 0, 0, 0, 0};    // Last perie value (speed) send by motors


    /* Odom structures */
    geometry_msgs::Quaternion odom_quat;
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster odom_broadcaster;

    /* Odom calc variables */
    float f_x_;
    float f_y_;
    float f_th_;
    float f_vxy_;
    float f_vth_;
    float f_dist_wtw_;    // Distance between left and right wheels 
    float f_diam_wheel_;  // Wheel diameter
    float f_delta_x_;
    float f_delta_y_;
    float f_delta_th_;
    float f_dt_;
   
    /* Vel calc variables */
    int i_rfw_nbtope;    // Encoder tic from buffer
    int i_rmw_nbtope;
    int i_rrw_nbtope;
    int i_lfw_nbtope;
    int i_lmw_nbtope;
    int i_lrw_nbtope;
    int i_rfw_perie;     // Perie value from buffer
    int i_rmw_perie;
    int i_rrw_perie;
    int i_lfw_perie;
    int i_lmw_perie;
    int i_lrw_perie;
    int i_countPerRev;   // nb de top horloge par top moteur
    int i_start_;        // count for initialization
    float rpm_top_speed_;    // used in comparaison bewteen top and peri
    float rpm_perie_speed_;    // used in comparaison bewteen top and peri

    /* Speed set into speed register */
    unsigned char uc_lfw_speed;
    unsigned char uc_lmw_speed; 
    unsigned char uc_lrw_speed;
    unsigned char uc_rfw_speed;
    unsigned char uc_rmw_speed;
    unsigned char uc_rrw_speed;

};

/*--------------*/
/* Constructeur */
/*--------------*/
DcmcRvMotorController::DcmcRvMotorController() {
    ps_iartemis = &s_rover;

    vfw_null.lfw = 0;// set null vector  
    vfw_null.lmw = 0;
    vfw_null.lrw = 0;
    vfw_null.rfw = 0;
    vfw_null.rmw = 0;
    vfw_null.rrw = 0;

    vf_rpm_vel = vfw_null;
    i_start_ = 0;

    /* callback, sub and pub initialization */
    rpm_sub = nh.subscribe("/motors/cmd", 10, &DcmcRvMotorController::rpm_callback, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    rpm_pub = nh.advertise<c_iartemis_base_controller::vfw_float32>("/motors/rpm_state", 10);
    top_pub = nh.advertise<std_msgs::String>("/motors/top", 10);
    perie_pub = nh.advertise<std_msgs::String>("/motors/perie", 10);
    controllerTimer_dcmc = nh.createTimer(ros::Duration(0.05), &DcmcRvMotorController::controllerTimerCallback, this, false);
    
    /* rover initialization (brake OFF, gear LOW, dir POS, speed NULL) */
    DcmcRvMotorController::rv_dmc_init_iartemis(&s_rover);
    DcmcRvMotorController::rv_dmc_set_brake_iartemis(E_RM_BRAKE_OFF, &s_rover);
    DcmcRvMotorController::rv_dmc_set_gear_iartemis( E_RM_GEAR_LOW, &s_rover);
    DcmcRvMotorController::rv_dmc_set_ls_on_iartemis(I_RM_FALSE, &s_rover);
    DcmcRvMotorController::rv_dmc_set_rs_on_iartemis(I_RM_FALSE, &s_rover);
    DcmcRvMotorController::rv_dmc_set_ls_dir_iartemis(E_RM_DMC_DIR_POS, &s_rover);
    DcmcRvMotorController::rv_dmc_set_rs_dir_iartemis(E_RM_DMC_DIR_POS, &s_rover);
    DcmcRvMotorController::rv_dmc_set_ls_speed_iartemis(vfw_null, &s_rover);
    DcmcRvMotorController::rv_dmc_set_rs_speed_iartemis(vfw_null, &s_rover);
    e_errstat = DcmcRvMotorController::rv_dmc_apply_iartemis(&s_rover);

    /* odom variables initialization */
    f_x_ = 0.0;
    f_y_ = 0.0;
    f_th_ = 0.0;
    f_vxy_ = 0.0;
    f_vth_ = 0.0;
    updateParameters();// set parameters from rover 

    /* Timer initialization */
    t_current_time = ros::Time::now();
    t_last_time = ros::Time::now();

}// DcmcRvMotorController


/*-------------*/
/* Destructeur */
/*-------------*/
/*DcmcRvMotorController::~DcmcRvMotorController(){
}// ~DcmcRvMotorController*/

/*----------------*/
/* Initialisation */
/*----------------*/
void DcmcRvMotorController::rv_dmc_init_iartemis(S_iartemis *ps_rover) {
    int i_idx;
    ps_rover->ps_file = fopen("/dev/ttyUSB0", "a+");
    assert(ps_rover->ps_file != NULL);
    for (i_idx = 0; i_idx < I_RM_NB_DMC; i_idx++) {
        dmc_init(&(ps_rover->ts_dmc[i_idx])); /* ! brake on */
        dmc_set_address('h' + 2 * i_idx, &(ps_rover->ts_dmc[i_idx]));
    }
}  // rv_dmc_init_iartemis

/*----------------------*/
/* Rover apply fonction */
/*----------------------*/
E_RM_ERROR DcmcRvMotorController::rv_dmc_apply_iartemis(S_iartemis *ps_rover) {
    int i_idx;
    E_RM_ERROR e_errstat;

    e_errstat = E_RM_ERROR_OK;
    if (ps_rover->b_dmc_modified) {
        for (i_idx = 0; !e_errstat && i_idx < I_RM_NB_DMC; i_idx++) {
            e_errstat = dmc_fprintf(ps_rover->ps_file, &(ps_rover->ts_dmc[i_idx]));  // evois des valeurs dans les
            // registres d'écriture
        }
        ps_rover->b_dmc_modified = 0;
    }
    return e_errstat;
}

/*-----------------------------------*/
/* Rover driving direction functions */
/*-----------------------------------*/
void DcmcRvMotorController::rv_dmc_set_ls_dir_iartemis(E_RM_DMC_DIR e_direction, S_iartemis *ps_rover) {
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
}  // rv_dmc_set_ls_dir_iartemis

void DcmcRvMotorController::rv_dmc_set_rs_dir_iartemis(E_RM_DMC_DIR e_direction, S_iartemis *ps_rover) {
    dmc_set_rw_dir(e_direction, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
    dmc_set_rw_dir(e_direction, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
    dmc_set_rw_dir(e_direction, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));

    ps_rover->e_dmc_rs_dir = e_direction;
    ps_rover->b_dmc_modified = 1;
}  // rv_dmc_set_rs_dir_iartemis

/*----------------------*/
/* Rover speed function */
/*----------------------*/
void DcmcRvMotorController::rv_dmc_set_ls_speed_iartemis(const c_iartemis_base_controller::vfw_float32 vf_rpm_set_l,
                                                      S_iartemis *ps_rover) {
    c_iartemis_base_controller::vfw_float32 vf_rpm_tmp = vf_rpm_set_l;
    // ROS_INFO("rpm: %.2f %.2f %.2f", vf_rpm_tmp.lfw, vf_rpm_tmp.lmw, vf_rpm_tmp.lrw);

    uc_lfw_speed = std::abs((unsigned char)vf_rpm_tmp.lfw);
    uc_lmw_speed = std::abs((unsigned char)vf_rpm_tmp.lmw);
    uc_lrw_speed = std::abs((unsigned char)vf_rpm_tmp.lrw);

    dmc_set_lw_speed(uc_lfw_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
    dmc_set_lw_speed(uc_lmw_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
    dmc_set_lw_speed(uc_lrw_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));

    ps_rover->uc_dmc_ls_speed = uc_lmw_speed;
    ps_rover->b_dmc_modified = 1;

}  // rv_dmc_set_ls_speed_iartemis

void DcmcRvMotorController::rv_dmc_set_rs_speed_iartemis(const c_iartemis_base_controller::vfw_float32 vf_rpm_set_r,
                                                      S_iartemis *ps_rover) {
    c_iartemis_base_controller::vfw_float32 vf_rpm_tmp = vf_rpm_set_r;
    // ROS_INFO("rpm:  %.2f %.2f %.2f", vf_rpm_tmp.rfw, vf_rpm_tmp.rmw, vf_rpm_tmp.rrw);

    uc_rfw_speed = std::abs((unsigned char)vf_rpm_tmp.rfw);
    uc_rmw_speed = std::abs((unsigned char)vf_rpm_tmp.rmw);
    uc_rrw_speed = std::abs((unsigned char)vf_rpm_tmp.rrw);

    dmc_set_rw_speed(uc_rfw_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
    dmc_set_rw_speed(uc_rmw_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
    dmc_set_rw_speed(uc_rrw_speed, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));

    ps_rover->uc_dmc_rs_speed = uc_rmw_speed;
    ps_rover->b_dmc_modified = 1;
}  // rv_dmc_set_rs_speed_iartemis

/*-------------------*/
/* Rover on function */
/*-------------------*/
void DcmcRvMotorController::rv_dmc_set_ls_on_iartemis(T_RM_BOOLEAN e_boolean, S_iartemis *ps_rover) {
    dmc_set_lw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
    dmc_set_lw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
    dmc_set_lw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));

    ps_rover->b_dmc_ls_on = e_boolean;
    ps_rover->b_dmc_modified = 1;
}  // rv_dmc_set_ls_on_iartemis

void DcmcRvMotorController::rv_dmc_set_rs_on_iartemis(T_RM_BOOLEAN e_boolean, S_iartemis *ps_rover) {
    dmc_set_rw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_FRONT]));
    dmc_set_rw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
    dmc_set_rw_on(e_boolean, &(ps_rover->ts_dmc[I_RM_iartemis_CARD_REAR]));

    ps_rover->b_dmc_ls_on = e_boolean;
    ps_rover->b_dmc_modified = 1;
}  // rv_dmc_set_rs_on_iartemis

/*----------------------*/
/* Rover brake function */
/*----------------------*/
void DcmcRvMotorController::rv_dmc_set_brake_iartemis(E_RM_BRAKE e_brake, S_iartemis *ps_rover) {
    int i_idx;
    for (i_idx = 0; i_idx < I_RM_NB_DMC; i_idx++) {
        dmc_set_lrw_brake(e_brake, &(ps_rover->ts_dmc[i_idx]));
    }
    ps_rover->e_brake = e_brake;
    ps_rover->b_dmc_modified = 1;
}  // rv_dmc_set_brake

/*---------------------*/
/* Rover gear function */
/*---------------------*/
void DcmcRvMotorController::rv_dmc_set_gear_iartemis(E_RM_GEAR e_gear, S_iartemis *ps_rover) {
    int i_idx;
    for (i_idx = 0; i_idx < I_RM_NB_DMC; i_idx++) {
        dmc_set_lrw_gear(e_gear, &(ps_rover->ts_dmc[i_idx]));
    }
    ps_rover->e_gear = e_gear;
    ps_rover->b_dmc_modified = 1;
}// rv_dmc_set_gear_iartemis

/*-----------------------------------------*/
/* Rover set speed absolute value function */
/*-----------------------------------------*/
void DcmcRvMotorController::rv_dmc_set_abs_speed(S_iartemis *ps_rover) {
    vf_rpm_vel_abs.lfw = fabs(vf_rpm_vel.lfw);
    vf_rpm_vel_abs.lmw = fabs(vf_rpm_vel.lmw);
    vf_rpm_vel_abs.lrw = fabs(vf_rpm_vel.lrw);
    vf_rpm_vel_abs.rfw = fabs(vf_rpm_vel.rfw);
    vf_rpm_vel_abs.rmw = fabs(vf_rpm_vel.rmw);
    vf_rpm_vel_abs.rrw = fabs(vf_rpm_vel.rrw);
}//

/*-----------------------*/
/* RPM Callback function */
/*-----------------------*/
void DcmcRvMotorController::rpm_callback(const c_iartemis_base_controller::vfw_float32 vf_rpm_cb) {
    E_RM_ERROR e_errstat;
    e_errstat = E_RM_ERROR_OK;
    vf_rpm_vel = vf_rpm_cb;
    rv_dmc_set_abs_speed(&s_rover);
    // ROS_INFO("rpm set speed before send:  %.5f %.5f %.5f %.5f %.5f %.5f", vf_rpm_vel.lfw, vf_rpm_vel.lmw, vf_rpm_vel.lrw, vf_rpm_vel.rfw, vf_rpm_vel.rmw, vf_rpm_vel.rrw);

    if (ps_iartemis->e_brake == E_RM_BRAKE_OFF) {    // can set a speed value only if brake = OFF
        if (!e_errstat) {                         // set direction from middle wheels sign
            e_rs_dir = ps_iartemis->e_dmc_rs_dir;
            if (vf_rpm_vel.rmw > 0) {
                e_rs_dir = E_RM_DMC_DIR_POS;
            } else if (vf_rpm_vel.rmw < 0) {
                e_rs_dir = E_RM_DMC_DIR_NEG;
            }

            e_ls_dir = ps_iartemis->e_dmc_ls_dir;
            if (vf_rpm_vel.lmw > 0) {
                e_ls_dir = E_RM_DMC_DIR_POS;
            } else if (vf_rpm_vel.lmw < 0) {
                e_ls_dir = E_RM_DMC_DIR_NEG;
            }
        }  // if

        if (!e_errstat) {
            /* check if a pause is needed before changing speed (2 stopframes if both sides change) */
            /* the check is done on middle wheel only because front and back weels speeds are calculated from middle wheel
             * speed (cf twist_to_motors.cpp) */
            if ((vf_rpm_vel.lmw > 0 && ps_iartemis->e_dmc_ls_dir == E_RM_DMC_DIR_NEG) ||
                (vf_rpm_vel.lmw < 0 && ps_iartemis->e_dmc_ls_dir == E_RM_DMC_DIR_POS)) {
                DcmcRvMotorController::rv_dmc_set_ls_on_iartemis(I_RM_FALSE, ps_iartemis);
                e_errstat = DcmcRvMotorController::rv_dmc_apply_iartemis(ps_iartemis);
                // set direction and speed on lw
                DcmcRvMotorController::rv_dmc_set_ls_dir_iartemis(e_ls_dir, ps_iartemis);
                DcmcRvMotorController::rv_dmc_set_ls_speed_iartemis(vf_rpm_vel_abs, ps_iartemis);

            } else if ((vf_rpm_vel.rmw > 0 && ps_iartemis->e_dmc_rs_dir == E_RM_DMC_DIR_NEG) ||
                       (vf_rpm_vel.rmw < 0 && ps_iartemis->e_dmc_rs_dir == E_RM_DMC_DIR_POS)) {
                DcmcRvMotorController::rv_dmc_set_rs_on_iartemis(I_RM_FALSE, ps_iartemis);
                e_errstat = DcmcRvMotorController::rv_dmc_apply_iartemis(ps_iartemis);
                // set direction and speed on rw
                DcmcRvMotorController::rv_dmc_set_rs_dir_iartemis(e_rs_dir, ps_iartemis);
                DcmcRvMotorController::rv_dmc_set_rs_speed_iartemis(vf_rpm_vel_abs, ps_iartemis);

            } else {
                // set direction and speed on both
                DcmcRvMotorController::rv_dmc_set_ls_dir_iartemis(e_ls_dir, &s_rover);
                DcmcRvMotorController::rv_dmc_set_ls_speed_iartemis(vf_rpm_vel_abs, &s_rover);
                DcmcRvMotorController::rv_dmc_set_ls_on_iartemis(I_RM_TRUE, &s_rover);
                DcmcRvMotorController::rv_dmc_set_rs_dir_iartemis(e_rs_dir, &s_rover);
                DcmcRvMotorController::rv_dmc_set_rs_speed_iartemis(vf_rpm_vel_abs, &s_rover);
                DcmcRvMotorController::rv_dmc_set_rs_on_iartemis(I_RM_TRUE, &s_rover);
                e_errstat = DcmcRvMotorController::rv_dmc_apply_iartemis(&s_rover);
            }
        }// if
    }// if_BRAKE_OFF
}

/*----------------------------------*/
/* Odom message generation function */
/*----------------------------------*/
void DcmcRvMotorController::generateOdomMsg(int i_vel_rw, int i_vel_lw) {
    f_dt_ = (t_current_time - t_last_time).toSec();

    float f_rw_value = DcmcRvMotorController::radsFromPerie(i_vel_rw);
    float f_lw_value = DcmcRvMotorController::radsFromPerie(i_vel_lw);

    f_vxy_ = (f_rw_value + f_lw_value)* f_diam_wheel_ / 2;
    f_vth_ = ((f_rw_value - f_lw_value) / f_dist_wtw_) * f_diam_wheel_;

    f_delta_x_ = f_vxy_ * cos(f_th_) * f_dt_;
    f_delta_y_ = f_vxy_ * sin(f_th_) * f_dt_;
    f_delta_th_ = f_vth_ * f_dt_;

    f_x_ += f_delta_x_;
    f_y_ += f_delta_y_;
    f_th_ += f_delta_th_;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(f_th_);

    odom_trans.header.stamp = t_current_time;
    odom_trans.header.frame_id = "wheel_odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = f_x_;
    odom_trans.transform.translation.y = f_y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = t_current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = f_x_;
    odom.pose.pose.position.y = f_y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = f_vxy_;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = 0.0;
    odom.twist.twist.angular.z = 0.0;
    odom.twist.twist.angular.z = f_vth_;

    odom_pub.publish(odom);
}  // generateOdomMsg


/*-------------------------*/
/* Timer Callback function */
/*-------------------------*/
void DcmcRvMotorController::controllerTimerCallback(const ros::TimerEvent &e) {
    int ti_nbtope[I_NB_WHEEL] = {0, 0, 0, 0, 0, 0};
    int ti_perie[I_NB_WHEEL] = {0, 0, 0, 0, 0, 0};
    int ti_vel_perie[I_NB_WHEEL] = {0, 0, 0, 0, 0, 0};

 
    /* Recovery of buffers values */
    dmc_fscanf(ps_iartemis->ps_file, &(ps_iartemis->ts_dmc[I_RM_iartemis_CARD_FRONT]));
    S_RM_DMC_RD_BUFF *ps_dmc_rdf_buff = &(ps_iartemis->ts_dmc[I_RM_iartemis_CARD_FRONT].s_rd_buff);

    i_lfw_nbtope = ps_dmc_rdf_buff->uc_ntope13 << 16 | ps_dmc_rdf_buff->uc_ntope12 << 8 | ps_dmc_rdf_buff->uc_ntope11;
    i_rfw_nbtope = ps_dmc_rdf_buff->uc_ntope23 << 16 | ps_dmc_rdf_buff->uc_ntope22 << 8 | ps_dmc_rdf_buff->uc_ntope21;

    i_lfw_perie = ps_dmc_rdf_buff->uc_perie12 << 8 | ps_dmc_rdf_buff->uc_perie11;
    i_rfw_perie = ps_dmc_rdf_buff->uc_perie22 << 8 | ps_dmc_rdf_buff->uc_perie21;

    dmc_fscanf(ps_iartemis->ps_file, &(ps_iartemis->ts_dmc[I_RM_iartemis_CARD_MIDDLE]));
    S_RM_DMC_RD_BUFF *ps_dmc_rdm_buff = &(ps_iartemis->ts_dmc[I_RM_iartemis_CARD_MIDDLE].s_rd_buff);

    i_lmw_nbtope = ps_dmc_rdm_buff->uc_ntope13 << 16 | ps_dmc_rdm_buff->uc_ntope12 << 8 | ps_dmc_rdm_buff->uc_ntope11;
    i_rmw_nbtope = ps_dmc_rdm_buff->uc_ntope23 << 16 | ps_dmc_rdm_buff->uc_ntope22 << 8 | ps_dmc_rdm_buff->uc_ntope21;

    i_lmw_perie = ps_dmc_rdm_buff->uc_perie12 << 8 | ps_dmc_rdm_buff->uc_perie11;
    i_rmw_perie = ps_dmc_rdm_buff->uc_perie22 << 8 | ps_dmc_rdm_buff->uc_perie21;

    dmc_fscanf(ps_iartemis->ps_file, &(ps_iartemis->ts_dmc[I_RM_iartemis_CARD_REAR]));
    S_RM_DMC_RD_BUFF *ps_dmc_rdb_buff = &(ps_iartemis->ts_dmc[I_RM_iartemis_CARD_REAR].s_rd_buff);

    i_lrw_nbtope = ps_dmc_rdb_buff->uc_ntope13 << 16 | ps_dmc_rdb_buff->uc_ntope12 << 8 | ps_dmc_rdb_buff->uc_ntope11;
    i_rrw_nbtope = ps_dmc_rdb_buff->uc_ntope23 << 16 | ps_dmc_rdb_buff->uc_ntope22 << 8 | ps_dmc_rdb_buff->uc_ntope21;

    i_lrw_perie = ps_dmc_rdb_buff->uc_perie12 << 8 | ps_dmc_rdb_buff->uc_perie11;
    i_rrw_perie = ps_dmc_rdb_buff->uc_perie22 << 8 | ps_dmc_rdb_buff->uc_perie21;

    //ROS_INFO(" Return Top %d %d %d %d %d %d ", i_lfw_nbtope, i_lmw_nbtope, i_lrw_nbtope, i_rfw_nbtope, i_rmw_nbtope,i_rrw_nbtope);
    //ROS_INFO(" Return Perie %i %i %i %i %i %i ", i_lfw_perie, i_lmw_perie,  i_lrw_perie, i_rfw_perie, i_rmw_perie, i_rrw_perie);

    /* Update time value*/    
    t_current_time = ros::Time::now();
    ros::Duration time = t_current_time - t_last_time;

    /* Put buffers values in tabs */
    ti_nbtope[0] = i_lfw_nbtope;
    ti_nbtope[1] = i_lmw_nbtope;
    ti_nbtope[2] = i_lrw_nbtope;
    ti_nbtope[3] = i_rfw_nbtope;
    ti_nbtope[4] = i_rmw_nbtope;
    ti_nbtope[5] = i_rrw_nbtope;


    ti_perie[0] = i_lfw_perie;
    ti_perie[1] = i_lmw_perie;
    ti_perie[2] = i_lrw_perie;
    ti_perie[3] = i_rfw_perie;
    ti_perie[4] = i_rmw_perie;
    ti_perie[5] = i_rrw_perie;

    /* Calc nb of encoder tic between two timer callback*/
    // 31250 top horloge/s
    for(int i = 0; i < I_NB_WHEEL; i++ ) {
        vi_nb_top_diff_[i] = ti_nbtope[i] - ti_lastEncMessage_[i];
    }// for

    /* Set sign of nb_top value according to direction */  
    if (e_ls_dir == E_RM_DMC_DIR_NEG) {
        for(int i = 0; i < 3; i++ ) {
            vi_nb_top_diff_[i] = - abs(vi_nb_top_diff_[i]);
        }// for
    }// if

    if (e_rs_dir == E_RM_DMC_DIR_NEG) {
        for(int i = 3; i < I_NB_WHEEL; i++ ) {
            vi_nb_top_diff_[i] = - abs(vi_nb_top_diff_[i]);
        }// for
    }// if

    /* Set sign of perie value according to direction */  
    if (e_ls_dir == E_RM_DMC_DIR_NEG) {
        for(int i = 0; i < 3; i++ ) {
            ti_perie[i] = - abs(ti_perie[i]);
        }// for
    }// if

    if (e_rs_dir == E_RM_DMC_DIR_NEG) {
        for(int i = 3; i < I_NB_WHEEL; i++ ) {
            ti_perie[i] = - abs(ti_perie[i]);
        }// for
    }// if

    /* Check motors state (perie and nb_top not update -> stop) and convert into rpm */
    for(int i = 0; i < I_NB_WHEEL; i++ ) {
        if (ti_nbtope[i] != ti_last3EncMessage_[i] || 
            ti_nbtope[i] != ti_last2EncMessage_[i] ||
            ti_nbtope[i] != ti_lastEncMessage_[i] ||
            ti_lastPerieMessage_[i] != ti_perie[i]) {
            ti_vel_perie[i] = ti_perie[i];
            tf_rpm_msg_[i] = rpmFromPerie (ti_vel_perie[i]);

        } else {
            ti_vel_perie[i] = 0;
            tf_rpm_msg_[i] = 0;
        }
        /*ROS_INFO("ti_nb_tope[%d] %d = %d",i, ti_nbtope[i], ti_last3EncMessage_[i]);
        ROS_INFO("ti_nb_tope[%d] %d = %d",i, ti_nbtope[i], ti_last2EncMessage_[i]);
        ROS_INFO("ti_nb_tope[%d] %d = %d",i, ti_nbtope[i], ti_lastEncMessage_[i]);
        ROS_INFO("ti_perie[%d] %d = %d",i, ti_perie[i], ti_lastPerieMessage_[i]);
        ROS_INFO("ti_vel_perie[%d] %d",i, ti_vel_perie[i] ); */
    }// for

    /* Set rpm_value in rpm_msg which must be published */
    rpm_msg.lfw = tf_rpm_msg_[0];
    rpm_msg.lmw = tf_rpm_msg_[1];
    rpm_msg.lrw = tf_rpm_msg_[2];
    rpm_msg.rfw = tf_rpm_msg_[3];
    rpm_msg.rmw = tf_rpm_msg_[4];
    rpm_msg.rrw = tf_rpm_msg_[5];

    /* Send rpm values (for c_pid_node) from motors on ROS topic */
    //ROS_INFO("rpm_msg send %.5f %.5f %.5f %.5f %.5f %.5f \n", rpm_msg.lfw, rpm_msg.rfw, rpm_msg.lmw, rpm_msg.rmw, rpm_msg.lrw, rpm_msg.rrw);
    //ROS_INFO ("DIR RIGHT = %d, DIR LEFT = %d", e_rs_dir, e_ls_dir); 

    if( i_start_ > 10 ) { // loop nb for initialisation
        rpm_pub.publish(rpm_msg);
    }//if
    i_start_++;

    /* Send odometry values from motors on ROS topics */
    DcmcRvMotorController::generateOdomMsg(ti_vel_perie[4], ti_vel_perie[1]);

    /* Publish top and perie value to be compared, used for studied wheels' behavior */
    rpm_top_speed_ = DcmcRvMotorController::rpmFromEncoderCount(vi_nb_top_diff_[4],time.toSec());// vi_nb_top_diff_[4];//
    rpm_perie_speed_ = DcmcRvMotorController::rpmFromPerie(ti_vel_perie[4]);
    std_msgs::String msg_top;
    std_msgs::String msg_perie;
    std::stringstream ss_top;
    std::stringstream ss_perie;
    ss_top << (rpm_top_speed_);
    ss_perie << (rpm_perie_speed_);
    msg_top.data=ss_top.str();
    msg_perie.data=ss_perie.str();
    top_pub.publish(msg_top); //rpm_speed_
    perie_pub.publish(msg_perie);//ti_vel_perie[4]

    /* Save and update previous values */
    memcpy(ti_last3EncMessage_, ti_last2EncMessage_, sizeof(ti_last3EncMessage_));
    memcpy(ti_last2EncMessage_, ti_lastEncMessage_, sizeof(ti_last2EncMessage_));
    memcpy(ti_lastEncMessage_, ti_nbtope, sizeof(ti_lastEncMessage_));
    memcpy(ti_lastPerieMessage_, ti_perie, sizeof(ti_lastPerieMessage_));
    t_last_time = t_current_time;

}  // timerCallback

/*--------------------------*/
/* Rads generation function */
/*--------------------------*/
float DcmcRvMotorController::radsFromEncoderCount(int i_count, double time) {
    float f_rads_value = (((float)i_count / (float)i_countPerRev) *  2*M_PI) / time ;
    return f_rads_value;

}  // rpmToRads

/*-------------------------*/
/* RPM generation function */
/*-------------------------*/
float DcmcRvMotorController::rpmFromEncoderCount(int i_count, double time) {
    float f_rpm_value = ((float)i_count / (float)i_countPerRev) / time * 60.0;
    return f_rpm_value;
}


/*---------------------------------*/
/* perie to rpm generation function */
/*---------------------------------*/
float DcmcRvMotorController::rpmFromPerie(int i_perie) {
    if ( i_perie != 0) { 
    //31250 (clock top/s) * 60 (s/min) / (2200 (motor top/rev) * i_perie (clock top / motor top) -> rev/min
    float f_rpm_value = ( 31250 * 60 ) / (2200 * (float) i_perie) ; 
    return f_rpm_value;
    }
    return 0.0;
}

/*------------------------------------*/
/* perie to rad/s generation function */
/*------------------------------------*/
float DcmcRvMotorController::radsFromPerie(int i_perie) {
    if ( i_perie != 0) { 
    float f_rad_value = ( 31250 *  M_PI ) /( 2200 * (float)i_perie ); 
    return f_rad_value;
    }
    return 0.0;
}

/*------------------*/
/* Update parameters*/
/*------------------*/
void DcmcRvMotorController::updateParameters() {
    if (!nh.getParam("count_per_rev", i_countPerRev))
        i_countPerRev = 2200;
    if (!nh.getParam("dist_wtw", f_dist_wtw_))
        f_dist_wtw_ =  0.935;    
    if (!nh.getParam("diam_wheel", f_diam_wheel_))
        f_diam_wheel_ = 0.38; 
}

/*------------------------*/
/* Turn off rover function*/
/*------------------------*/
void DcmcRvMotorController::set_stop_state() {
    DcmcRvMotorController::rv_dmc_set_ls_speed_iartemis(vfw_null,&s_rover);
    DcmcRvMotorController::rv_dmc_set_ls_on_iartemis(I_RM_FALSE, &s_rover);
    DcmcRvMotorController::rv_dmc_set_rs_speed_iartemis(vfw_null, &s_rover);
    DcmcRvMotorController::rv_dmc_set_rs_on_iartemis(I_RM_FALSE, &s_rover);
    e_errstat = DcmcRvMotorController::rv_dmc_apply_iartemis(&s_rover);
}

/*--------------*/
/* sigintHandler*/
/*--------------*/
static DcmcRvMotorController *ps_motor_controller;



void sigintHandlerEnd(int i_sig) {
    ps_motor_controller->set_stop_state();

    ROS_INFO("NODE END");
    ros::shutdown();
}

/*------*/
/* Main */
/*------*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "dcmc_rv_motor_controller");
    DcmcRvMotorController motor_controller;
    ps_motor_controller = &motor_controller;
    signal(SIGINT, sigintHandlerEnd);
    ros::spin();

    return 0;
}
