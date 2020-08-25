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
#include <iostream>

// Test du parallélisme à l'intérieur d'un node.
// Conclusion : il n'y a pas de parallélisme à l'intérieur d'un même node.
// Les callback associés aux messages /rpm_csg sont reçus après la fin du Timer.
// rostopic pub -r 1 /rpm_csg c_iartemis_base_controller/vfw_float32 "{lfw: 0.0, lmw: 0.0, lrw: 0.0, rfw: 0.0, rmw: 0.0, rrw: 0.0}"


class XrPidNode {
  public:
    XrPidNode();
    ~XrPidNode();

  private:
    void state_callback(const c_iartemis_pid::vfw_float32::ConstPtr &msg);
    void csg_callback(const c_iartemis_pid::vfw_float32::ConstPtr &msg);
    void controller_timer_callback(const ros::TimerEvent &e);

    ros::Subscriber sub_rpm_csg;
    ros::Subscriber sub_rpm_state;

    ros::NodeHandle nh;
    ros::Timer controllerTimer;
};

XrPidNode::XrPidNode() {
    sub_rpm_csg = nh.subscribe("/rpm_csg", 10, &XrPidNode::csg_callback, this);
    //sub_rpm_state = nh.subscribe("/motors/rpm_state", 10, &XrPidNode::state_callback, this);
    //controllerTimer = nh.createTimer(ros::Duration(10.0), &XrPidNode::controller_timer_callback, this, false);
    controllerTimer = nh.createTimer(ros::Duration(2.0), &XrPidNode::controller_timer_callback, this, false);
}// XrPidNode

XrPidNode::~XrPidNode() {
}//~XrPidNode

void XrPidNode::state_callback(const c_iartemis_pid::vfw_float32::ConstPtr &msg) {
    return;
}// state_callback

void XrPidNode::csg_callback(const c_iartemis_pid::vfw_float32::ConstPtr &msg) {
    std::cout << "Debut XrPidNode::csg_callback" << std::endl; 
    usleep(1000*1000*5); // sleep for 5 seconds
    std::cout << "Fin XrPidNode::csg_callback" << std::endl; 
    return;
}// csg_callback



void XrPidNode::controller_timer_callback(const ros::TimerEvent &e){
    std::cout << "Debut XrPidNode::controller_timer_callback" << std::endl; 
    //usleep(1000*1000*5); // sleep for 5 seconds
    std::cout << "Fin XrPidNode::controller_timer_callback" << std::endl; 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xr_pid_node");
    XrPidNode xr_pid_node;

    ros::spin();

    return 0;
}
