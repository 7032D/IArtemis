/*****************************************************************************/
/*!
 * @file     ./c_pid_controller.cpp
 * @version  1.0
 * @date     Creation    : 20/03/2016
 * @date     Modified    :
 * @author   Florian MARTIN (c)
 * @project  iartemis ROS
 * @system   DRVPR
 * @brief    To handle generating valid output
 * @note     inspired from playground.arduino.cc/Code/PIDLibrary
 ***************************************************************************/
#include "ros/ros.h"
#include <string.h>
#include <cmath>

#define I_NB_WHEEL 6
#define sign(a) ( (a) < 0 )

static float gtf_zeros[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

class cPidController {
  public:
    cPidController();
    ~cPidController();

    void set_target(float tf_target[]);
    void set_input(float tf_input[]);
    void setTunings(float tf_kp[], float tf_ki[], float tf_kd[]);

    float *getTarget();
    float *getInput();

    float *compute();
    void reset();
    void debug();

  private:
    // Gains controlleur PID
    float tf_kp_[I_NB_WHEEL] = {0.65, 0.6, 0.6, 0.6, 0.6, 0.6};//lfw lmw lrw rfw rmw rrw
    float tf_ki_[I_NB_WHEEL] = {6.0057, 5.5437, 5.5437, 5.5437, 5.5437, 5.5437};
    float tf_kd_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    float tf_input_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};       // entree (mesure)
    float tf_target_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      // consigne
    float tf_prev_target_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // consigne précédente
    float tf_output_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      // sortie
    float tf_prev_ouput_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // sortie précédente

    // Memorisation des valeurs iteration n-1
    float tf_prev_error_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ros::Time prev_time_;

    float tf_i_total_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    float tf_min_limit_rpm_ = -4.0;
    float tf_max_limit_rpm_ = 4.0;

};


cPidController::cPidController() {
    prev_time_ = ros::Time::now();
};

cPidController::~cPidController() {
    memcpy(tf_target_,     gtf_zeros, sizeof(gtf_zeros));
    reset();
}

void cPidController::set_input(float tf_input[]) {
    memcpy(tf_input_, tf_input, sizeof(tf_input_));
    ROS_INFO("input get  %.2f",tf_input_[4]);
    return;
}

void cPidController::set_target(float tf_target[]) {
    ROS_INFO("tf_target[4] %f", tf_target[4]);
    memcpy(tf_prev_target_, tf_target_, sizeof(tf_prev_target_));
    memcpy(tf_target_, tf_target, sizeof(tf_target_));
    return;
}

void cPidController::setTunings(float tf_kp[], float tf_ki[], float tf_kd[]) {
    memcpy(tf_kp_, tf_kp, sizeof(tf_kp_));
    memcpy(tf_ki_, tf_ki, sizeof(tf_ki_));
    memcpy(tf_kd_, tf_kd, sizeof(tf_kd_));
    reset();
    return;
}

float *cPidController::getInput() {
    return tf_input_;
}

float *cPidController::getTarget() {
    return tf_target_;
}

float *cPidController::compute() {
    ros::Time now = ros::Time::now();
    ros::Duration change = now - prev_time_;

    float tf_error[I_NB_WHEEL] = {0,0,0,0,0,0};
    float *tf_d_value = new float[I_NB_WHEEL];
    float tf_target_calc_[I_NB_WHEEL]; //used for target sign change

    memcpy(tf_target_calc_, tf_target_, sizeof(tf_target_calc_));

    for(int j = 0; j < I_NB_WHEEL; j++ ) {
        if( fabs(tf_target_calc_[j]) > 0.001 ) {
            ROS_INFO("target %.2f",tf_target_calc_[j]);
            ROS_INFO("input  %.2f",tf_input_[j]);
            tf_error[j] = tf_target_calc_[j] - tf_input_[j];

            // compute integral component
            tf_i_total_[j] += (tf_error[j] * tf_ki_[j]) * change.toSec();

            // compute derivative component
            tf_d_value[j] = tf_kd_[j] * (tf_error[j] - tf_prev_error_[j]) / change.toSec();

            /* do the full calculation */
            tf_output_[j] = - tf_kp_[j] * tf_input_[j] + tf_i_total_[j] + tf_d_value[j];

            /* required values for next round */
            tf_prev_error_[j] = tf_error[j];
        }
        else{  
            tf_error[j] = 0.0; //force to 0 for the ROS_INFO
            tf_d_value[j] = 0.0;
            tf_i_total_[j] = 0.0;
            tf_output_[j] = 0.0;
            tf_prev_error_[j] = 0.0;
        }

        if (sign(tf_target_calc_[j]) != sign(tf_output_[j])) {
            tf_output_[j] = 0;
        }// if         

        /* debug */
        //ROS_INFO("P %.2f; I %.2f; D %.2f; Out %.2f, ki %.2f", tf_kp_[j] * tf_input_[j], tf_i_total_[j], tf_d_value[j], tf_output_[j], tf_ki_[j]);
    }// for

    prev_time_ = now;

    delete[] tf_d_value;

    return tf_output_;
}

void cPidController::debug(){
    for(int j = 0; j < I_NB_WHEEL; j++ ) {
        printf("%f ", tf_output_[j]);
    }
    printf("\n");
}


void cPidController::reset() {
    memcpy(tf_input_,      gtf_zeros, sizeof(gtf_zeros));
    memcpy(tf_output_,     gtf_zeros, sizeof(gtf_zeros));
    memcpy(tf_prev_error_, gtf_zeros, sizeof(gtf_zeros));
    return;
}
