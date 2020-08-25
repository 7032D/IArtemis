/*****************************************************************************/
/*!
 * @file     ./c_pid_node.cpp
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
#include "c_pid_controller.cpp"
#include "c_iartemis_pid/vfw_float32.h"
#include "c_iartemis_pid/k_coeff.h"
#include "std_msgs/Float32.h"
#include <unistd.h>
#include <math.h>

#define I_NB_WHEEL 6
#define sign(a) ( (a) < 0 )

class cPidNode {
  public:
    cPidNode();
    ~cPidNode();

  private:
    void update_parameters();
    bool set_tunings_service_callback(c_iartemis_pid::k_coeff::Request &req, c_iartemis_pid::k_coeff::Response &res);
    void state_callback(const c_iartemis_pid::vfw_float32::ConstPtr &msg);
    void csg_callback(const c_iartemis_pid::vfw_float32::ConstPtr &msg);
    float constrain(float value, float min, float max);
    float map_rpm_to_output(float rpm);
    void controller_timer_callback(const ros::TimerEvent &e);

    ros::Subscriber sub_rpm_csg;
    ros::Subscriber sub_rpm_state;
    ros::Publisher pub_cmd_motor_controller;
    ros::ServiceServer tuningService;

    ros::NodeHandle nh, private_nh;
    ros::Timer controllerTimer;
    
    cPidController controller_;

    float f_kp_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float f_ki_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float f_kd_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float tf_target_msg_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //used out of set_target function for check before sign change
    float tf_tmp_target_msg_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //used for target sign change
    float tf_prev_target_msg_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //used for target sign change
    float tf_input_msg_[I_NB_WHEEL] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //used out of set_input function for check before sign change

    double d_debug; 
    int i_max_RPM_;
    int i_max_cmd_output_;
    int i_modif_;
    int i_nb_frame_wait_;
    bool b_check_;
};

cPidNode::cPidNode() {
    // set the handler for the node's private namespace
    private_nh = ros::NodeHandle("~");
    i_max_RPM_ = 5;
    i_max_cmd_output_ = 255;
    i_modif_ = 0;
    i_nb_frame_wait_ = 0;
    b_check_ = 0;

    sub_rpm_csg = nh.subscribe("/rpm_csg", 10, &cPidNode::csg_callback, this);
    sub_rpm_state = nh.subscribe("/motors/rpm_state", 10, &cPidNode::state_callback, this);
    pub_cmd_motor_controller = nh.advertise<c_iartemis_pid::vfw_float32>("/motors/cmd", 10);
    tuningService = nh.advertiseService("set_tunings", &cPidNode::set_tunings_service_callback, this);
    controllerTimer = nh.createTimer(ros::Duration(0.05), &cPidNode::controller_timer_callback, this, false);

    update_parameters();

    controller_ = cPidController();
}// cPidNode

cPidNode::~cPidNode() {
    c_iartemis_pid::vfw_float32 motor_cmd_msg_null; // send in publisher
    motor_cmd_msg_null.lfw = 0.0;
    motor_cmd_msg_null.lmw = 0.0;
    motor_cmd_msg_null.lrw = 0.0;
    motor_cmd_msg_null.rfw = 0.0;
    motor_cmd_msg_null.rmw = 0.0;
    motor_cmd_msg_null.rrw = 0.0;
    
    memcpy(f_kp_, gtf_zeros, sizeof(gtf_zeros));
    memcpy(f_ki_, gtf_zeros, sizeof(gtf_zeros));
    memcpy(f_kd_, gtf_zeros, sizeof(gtf_zeros));
    memcpy(tf_target_msg_, gtf_zeros, sizeof(gtf_zeros));
    memcpy(tf_tmp_target_msg_, gtf_zeros, sizeof(gtf_zeros));
    memcpy(tf_prev_target_msg_, gtf_zeros, sizeof(gtf_zeros));
    memcpy(tf_input_msg_, gtf_zeros, sizeof(gtf_zeros));

    pub_cmd_motor_controller.publish(motor_cmd_msg_null);
}//~cPidNode

void cPidNode::update_parameters() {
        f_kp_[0] = 1.0;
        f_kp_[1] = 1.0;
        f_kp_[2] = 1.0;
        f_kp_[3] = 1.0;//0.45
        f_kp_[4] = 1.0;//0.4
        f_kp_[5] = 1.0;

        f_ki_[0] = 9.2464;
        f_ki_[1] = 9.2464;
        f_ki_[2] = 9.2464;
        f_ki_[3] = 9.2464;//3.7
        f_ki_[4] = 9.2464;//3.7
        f_ki_[5] = 9.2464;

        f_kd_[0] = 0;
        f_kd_[1] = 0;
        f_kd_[2] = 0;
        f_kd_[3] = 0;
        f_kd_[4] = 0;
        f_kd_[5] = 0;
    return;
}// update_parameters

bool cPidNode::set_tunings_service_callback(c_iartemis_pid::k_coeff::Request &req, c_iartemis_pid::k_coeff::Response &res) {
    controller_.setTunings(&req.kp, &req.ki, &req.kd);

    return true;
}// set_tunings_service_callback

void cPidNode::state_callback(const c_iartemis_pid::vfw_float32::ConstPtr &msg) {

    tf_input_msg_[0]= msg->lfw;
    tf_input_msg_[1]= msg->lmw;
    tf_input_msg_[2]= msg->lrw;
    tf_input_msg_[3]= msg->rfw;
    tf_input_msg_[4]= msg->rmw;
    tf_input_msg_[5]= msg->rrw;
    controller_.set_input(tf_input_msg_);

    return;
}// state_callback

void cPidNode::csg_callback(const c_iartemis_pid::vfw_float32::ConstPtr &msg) {
    
    memcpy(tf_prev_target_msg_, tf_target_msg_, sizeof(tf_prev_target_msg_));
    tf_target_msg_[0]= msg->lfw;
    tf_target_msg_[1]= msg->lmw;
    tf_target_msg_[2]= msg->lrw;
    tf_target_msg_[3]= msg->rfw;
    tf_target_msg_[4]= msg->rmw;
    tf_target_msg_[5]= msg->rrw;
    memcpy(tf_tmp_target_msg_, tf_target_msg_, sizeof(tf_tmp_target_msg_));
    controller_.set_target(tf_target_msg_);
    
    return;
}// csg_callback

float cPidNode::constrain(float value, float min, float max) {
    value = fmin(value, max);
    value = fmax(value, min);

    return value;
}// constrain

float cPidNode::map_rpm_to_output(float f_rpm) {
    // convert from a desired RPM value to a motor command (-255 - 255)
    float f_in_min = (float) -(i_max_RPM_);
    float f_in_max = (float) i_max_RPM_;
    float f_out_min = (float) -(i_max_cmd_output_);
    float f_out_max = (float) i_max_cmd_output_;

    return  ((f_rpm - f_in_min) * (f_out_max - f_out_min) / (f_in_max - f_in_min)) + f_out_min;
}// map_rpm_to_output

void cPidNode::controller_timer_callback(const ros::TimerEvent &e) {
    // can be improved by a separation between left and right side in zero step loop
    // if left and right side signs change together need all wheels at zero before original target
    float * tf_motor_cmd_calc;
    float tf_motor_cmd_msg[I_NB_WHEEL] = {0,0,0,0,0,0};    
    c_iartemis_pid::vfw_float32 motor_cmd_msg; // send in publisher

    for (int i=0; i<I_NB_WHEEL; i++) {
        // if target sign change
        if (sign(tf_prev_target_msg_[i]) != sign(tf_target_msg_[i])){
            tf_tmp_target_msg_[i] = 0.0;
            i_modif_ = 1;// enter in change sign loop
        }// if
    }// for
    
    if (i_modif_ == 1) {
        //ROS_INFO("b_modif = 1");
        controller_.set_target(tf_tmp_target_msg_);// apply new target with zero step 
        memcpy(tf_prev_target_msg_, tf_target_msg_, sizeof(tf_prev_target_msg_));// original target (without zero step) saved
        i_modif_ = 2;
    }// if

    if (i_modif_ == 2) {
        b_check_ = 0;
        //ROS_INFO("b_modif = 2");
        for (int k=0; k<I_NB_WHEEL; k++) {
            // if zero step not reaches
            if (tf_tmp_target_msg_[k] == 0.0 && fabs(tf_input_msg_[k]) > 0.001 ) {
                //ROS_INFO("tf_target_msg_ [%i] = %f", k, tf_target_msg_[k]);
                b_check_ = 1;
            }// if
        }// for

        // loop exit
        if (b_check_ == 0){
            if (i_nb_frame_wait_ == 3){
                controller_.reset(); 
                controller_.set_target(tf_target_msg_);// apply orginal target
                i_modif_ = 0;// reinitialize change loop
                i_nb_frame_wait_ = 0;
            }// if
            else {
                // tempo
                i_nb_frame_wait_++;
                ROS_INFO("i_nb_frame++");
            }
        }// if
    }// if

    tf_motor_cmd_calc = controller_.compute();
    controller_.debug();

    for (int i=0; i<I_NB_WHEEL; i++) {
        tf_motor_cmd_msg[i] = map_rpm_to_output(tf_motor_cmd_calc[i]);
        tf_motor_cmd_msg[i] = constrain(tf_motor_cmd_msg[i], - i_max_cmd_output_, i_max_cmd_output_); 
    }// for

    
//TODO vérif sens csg et cmd
    motor_cmd_msg.lfw = tf_motor_cmd_msg[0];
    motor_cmd_msg.lmw = tf_motor_cmd_msg[1];
    motor_cmd_msg.lrw = tf_motor_cmd_msg[2];
    motor_cmd_msg.rfw = tf_motor_cmd_msg[3];
    motor_cmd_msg.rmw = tf_motor_cmd_msg[4];
    motor_cmd_msg.rrw = tf_motor_cmd_msg[5];
    
    pub_cmd_motor_controller.publish(motor_cmd_msg);

    for (int h=0; h<I_NB_WHEEL; h++) {
        ROS_INFO("PID computed %.2f, Message send %.2f", tf_motor_cmd_calc[h], tf_motor_cmd_msg[h]);
    } // for
     
    ROS_INFO("debug %f", d_debug);

    return;
}// controller_timer_callback

int main(int argc, char **argv) {
    ros::init(argc, argv, "c_pid_node");
    cPidNode c_pid_node;

    ros::spin();

    return 0;
}
