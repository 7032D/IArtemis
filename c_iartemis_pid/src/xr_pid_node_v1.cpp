/*****************************************************************************/
/*!
 * @file     ./xr_pid_node.cpp
 * @version  1.0
 * @date     Creation    : 20/03/2016
 * @date     Modified    :
 * @author   Florian MARTIN (c)
 * @project  iartemis ROS
 * @system   DRVPR
 * @brief    Subscribe to encoder RPM topic and RPM setpoint topic and publishe to motor topic
 * @note     A service for setting the tuning parameters is usefull beacause you don't have to restart the launch file
 * @note     for update values kp, kd, ki.
 ***************************************************************************/

#include "ros/ros.h"
#include "c_iartemis_pid/vfw_float32.h"
#include "c_iartemis_pid/k_coeff.h"
#include "std_msgs/Float32.h"
#include <unistd.h>
#include <math.h>

#define I_NB_WHEEL 6

// Note :
// Il  n'y a pas de parallélisme à l'intérieur d'un node ROS.

// TODO :
// Mettre les noms des topics dans un fichier yaml ou bien en constantes statiques dans le Code
// Mettre les valeurs MIN et MAX RPM et la données de caractérisations dans un fichier yaml
// Mettre le seuil de 0.001 dans le fichier yaml ou en constante dans un fichier


bool isSameSign(const float f_a, const float f_b){
   return (f_a * f_b) >= 0.0;
}


float constrain(float f_value, float f_min, float f_max) {
    float f;
    f = fmin(f_value, f_max);
    f = fmax(f, f_min);
    return f;
}



class XrPidNode {
  public:
    XrPidNode();
    ~XrPidNode();

  private:
    void motorsStateCallback(const c_iartemis_pid::vfw_float32::ConstPtr &msg);
    void motorsTargetCallback(const c_iartemis_pid::vfw_float32::ConstPtr &msg);
    void pidTimer(const ros::TimerEvent &e);
    void pidCompute();

    void ros6Float32ToArray(float tf_array[I_NB_WHEEL], const c_iartemis_pid::vfw_float32::ConstPtr &msg);
    //void arrayToRos6Float32(const c_iartemis_pid::vfw_float32::ConstPtr &msg, float tf_array[I_NB_WHEEL]);
    void arrayToRos6Float32(c_iartemis_pid::vfw_float32 &msg, const float tf_array[I_NB_WHEEL]);
    float mapRpmToRaw(float rpm);

    ros::NodeHandle o_nh_;
    ros::Publisher  o_pub_motors_cmd_raw_;
    ros::Subscriber o_sub_csg_rpm;
    ros::Subscriber o_sub_state_rpm;
    ros::Time       o_prev_pid_time_;
    ros::Timer      o_timer;

    float tf_motors_kp_[I_NB_WHEEL] = {0.6500, 0.6000, 0.6000, 0.6000, 0.6000, 0.6000};
    float tf_motors_ki_[I_NB_WHEEL] = {6.0057, 5.5437, 5.5437, 5.5437, 5.5437, 5.5437};
    //float tf_motors_kp_[I_NB_WHEEL] = {0.4, 0.4, 0.4, 0.4, 0.0, 0.4};
    //float tf_motors_ki_[I_NB_WHEEL] = {3.8, 3.7, 3.7, 3.7, 1.0, 3.7};
    float tf_motors_kd_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    float tf_motors_state_rpm_[I_NB_WHEEL]       = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // current speed of motors in rpm
    float tf_motors_cmd_rpm_[I_NB_WHEEL]         = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // speed in rpm sent to motors
    float tf_motors_user_target_rpm_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // target speed asked by user
    float tf_motors_pid_target_rpm_[I_NB_WHEEL]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // target speed used in pid, for dealing with sign changes

    float tf_pid_intergral_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float tf_pid_prev_error_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    int i_max_RPM_;
    int i_max_cmd_output_;
};



XrPidNode::XrPidNode() {
    i_max_RPM_        = 5;
    i_max_cmd_output_ = 255;

    o_sub_csg_rpm = o_nh_.subscribe("/rpm_csg",          10, &XrPidNode::motorsTargetCallback, this);
    o_sub_state_rpm = o_nh_.subscribe("/motors/rpm_state", 10, &XrPidNode::motorsStateCallback,  this);

    o_pub_motors_cmd_raw_ = o_nh_.advertise<c_iartemis_pid::vfw_float32>("/motors/cmd", 10);

    o_timer = o_nh_.createTimer(ros::Duration(0.05), &XrPidNode::pidTimer, this, false);
    //o_timer = o_nh_.createTimer(ros::Duration(0.5), &XrPidNode::pidTimer, this, false);
}



XrPidNode::~XrPidNode() {
    c_iartemis_pid::vfw_float32 o_motors_cmd_vfw_null; // send in publisher
    o_motors_cmd_vfw_null.lfw = 0.0;
    o_motors_cmd_vfw_null.lmw = 0.0;
    o_motors_cmd_vfw_null.lrw = 0.0;
    o_motors_cmd_vfw_null.rfw = 0.0;
    o_motors_cmd_vfw_null.rmw = 0.0;
    o_motors_cmd_vfw_null.rrw = 0.0;

    //memcpy(tf_motors_kp_, gtf_zeros, sizeof(gtf_zeros));
    //memcpy(tf_motors_ki_, gtf_zeros, sizeof(gtf_zeros));
    //memcpy(tf_motors_kd_, gtf_zeros, sizeof(gtf_zeros));
    //memcpy(tf_motors_user_target_rpm_, gtf_zeros, sizeof(gtf_zeros));
    //memcpy(tf_motors_pid_target_rpm_, gtf_zeros, sizeof(gtf_zeros));
    //memcpy(tf_motors_state_rpm_, gtf_zeros, sizeof(gtf_zeros));

    o_pub_motors_cmd_raw_.publish(o_motors_cmd_vfw_null);
}//~XrPidNode



void XrPidNode::ros6Float32ToArray(float tf_array[I_NB_WHEEL], const c_iartemis_pid::vfw_float32::ConstPtr &msg) {
    tf_array[0]= msg->lfw;
    tf_array[1]= msg->lmw;
    tf_array[2]= msg->lrw;
    tf_array[3]= msg->rfw;
    tf_array[4]= msg->rmw;
    tf_array[5]= msg->rrw;
    return;
}


void XrPidNode::arrayToRos6Float32(c_iartemis_pid::vfw_float32 &msg, const float tf_array[I_NB_WHEEL]) {
    msg.lfw = tf_array[0];
    msg.lmw = tf_array[1];
    msg.lrw = tf_array[2];
    msg.rfw = tf_array[3];
    msg.rmw = tf_array[4];
    msg.rrw = tf_array[5];
    return;
}


void XrPidNode::motorsStateCallback(const c_iartemis_pid::vfw_float32::ConstPtr &msg) {
    ros6Float32ToArray(tf_motors_state_rpm_, msg);
    return;
}



void XrPidNode::motorsTargetCallback(const c_iartemis_pid::vfw_float32::ConstPtr &msg) {
    ros6Float32ToArray(tf_motors_user_target_rpm_, msg);
    return;
}





float XrPidNode::mapRpmToRaw(float f_rpm) {
    // convert from a desired RPM value to a motor command (-255 - 255)
    float f_in_min = (float) -(i_max_RPM_);
    float f_in_max = (float) i_max_RPM_;
    float f_out_min = (float) -(i_max_cmd_output_);
    float f_out_max = (float) i_max_cmd_output_;

    return  ((f_rpm - f_in_min) * (f_out_max - f_out_min) / (f_in_max - f_in_min)) + f_out_min;
}// mapRpmToRaw



void XrPidNode::pidCompute() {
    float f_pid_error;
    float f_pid_proportional;
    float f_pid_derivative;

    ros::Time o_now = ros::Time::now();
    //ros::Duration o_deltatime = o_now - o_prev_pid_time_;
    float d_deltatime = (o_now - o_prev_pid_time_).toSec();

    for(int j = 0; j < I_NB_WHEEL; j++ ) {
        if( fabs(tf_motors_pid_target_rpm_[j]) > 0.001 ) {
            // compute error
            f_pid_error = tf_motors_pid_target_rpm_[j] - tf_motors_state_rpm_[j];

            // compute proportional component (warning: not a classic PID)
            f_pid_proportional = -(tf_motors_kp_[j] * tf_motors_state_rpm_[j]);

            // compute integral component
            tf_pid_intergral_[j] += tf_motors_ki_[j] * f_pid_error * d_deltatime;

            // compute derivative component
            f_pid_derivative = tf_motors_kd_[j] * (f_pid_error - tf_pid_prev_error_[j]) / d_deltatime;

            /* compute command */
            tf_motors_cmd_rpm_[j] = f_pid_proportional + tf_pid_intergral_[j] + f_pid_derivative;

            /* store required values for next round */
            tf_pid_prev_error_[j] = f_pid_error;
        }
        else{
            // ask for a speed of 0 => stop motors and reset pid proportional and derivative component
            f_pid_error = 0.0;
            f_pid_proportional = 0.0;
            tf_pid_intergral_[j] = 0.0;
            f_pid_derivative = 0.0;
            tf_motors_cmd_rpm_[j] = 0.0;
            tf_pid_prev_error_[j] = 0.0;
        }

        // check sign of command versus sign of target to protect motors, force 0.0 if needed
        if ( !isSameSign(tf_motors_pid_target_rpm_[j], tf_motors_cmd_rpm_[j]) ) {
            tf_motors_cmd_rpm_[j] = 0.0;
        }

        /* debug */
        ROS_INFO("PID Compute (rpm) [%d] => User Target: %.2f; Pid Target: %.2f; State: %.2f; Command : %.2f; P: %.2f; I: %.2f; D: %.2f;",
                j, tf_motors_user_target_rpm_[j], tf_motors_pid_target_rpm_[j], tf_motors_state_rpm_[j], tf_motors_cmd_rpm_[j],
                f_pid_proportional, tf_pid_intergral_[j], f_pid_derivative);
    }

    o_prev_pid_time_ = o_now;

    return;
}



void XrPidNode::pidTimer(const ros::TimerEvent &e) {
    float tf_motors_cmd_raw[I_NB_WHEEL];
    c_iartemis_pid::vfw_float32 o_motors_cmd_vfw; // send in publisher

    // set target for pid, set 0.0 if sign of target and sign of state are differents
    for (int i=0; i<I_NB_WHEEL; i++) {
        if (isSameSign(tf_motors_user_target_rpm_[i], tf_motors_state_rpm_[i])){
            tf_motors_pid_target_rpm_[i] = tf_motors_user_target_rpm_[i];
        }
        else{
            tf_motors_pid_target_rpm_[i] = 0.0;
        }
    }

    // compute one step of pid
    pidCompute();

    // check and remap commands to motors
    for (int i=0; i<I_NB_WHEEL; i++) {
        tf_motors_cmd_raw[i] = mapRpmToRaw(tf_motors_cmd_rpm_[i]);
        tf_motors_cmd_raw[i] = constrain(tf_motors_cmd_raw[i], -i_max_cmd_output_, i_max_cmd_output_);
    }

    // convert array to ros
    arrayToRos6Float32(o_motors_cmd_vfw, tf_motors_cmd_raw);

    // send commands to motors
    o_pub_motors_cmd_raw_.publish(o_motors_cmd_vfw);

    for (int i=0; i<I_NB_WHEEL; i++) {
        ROS_INFO("PID Timer (raw) [%d] => Command %.2f", i, tf_motors_cmd_raw[i]);
    }

    return;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "xr_pid_node");

    XrPidNode xr_pid_node;

    ros::spin();

    return 0;
}
