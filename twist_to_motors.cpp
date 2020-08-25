/*****************************************************************************/
/*!
 * @file     ./twist_to_motors.c
 * @version  1.0
 * @date     Creation    : 16/03/2016
 * @date     Modified    :
 * @author   Florian Martin
 * @project  iartemis ROS
 * @system   DRVPR
 * @brief    Conversion from twist messages (linear/angular) to motors messages (left/right)
 ***************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"

#include <sstream>
#include <geometry_msgs/Twist.h>


/**
* Déclarations
*/
int ticks_since_target, timeout_ticks, tick_since_target, argc, rate;
double dx, dr, dy, dist_lrw, rw_vel, lw_vel;
char** argv;
ros::Publisher pub_rmotor;
ros::Publisher pub_lmotor;
ros::Subscriber sub;
ros::NodeHandle n;

/**
* Callback function called when a new message has arrived 
*/
void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
  ROS_INFO("twist: [%f]", vel_cmd.linear.x);
  ticks_since_target = 0;
  double dx = vel_cmd.linear.x;	// commande linéaire
  double dr = vel_cmd.angular.z;	// commande angulaire
  double dy = vel_cmd.linear.y;
}// twistCallback


/**
*Initialisation
*/
void init()
{
  ros::init(argc, argv, "twist_to_motors");

  dist_lrw = n.param("/dist_lrw", 0.935);// distance entre roue droite et roue gauche, 146 + 790 = lr + daxe

  rate = n.param("/rate",50);
  timeout_ticks = n.param("/timeout_ticks", 2);
  lw_vel = 0;
  rw_vel = 0;
}// init


/**
* Publish the "twist to motor" conversion once
* dx = (rw_vel + lw_vel) / 2
* dr = (rw_vel - lw_vel) / 2
*/
void spinOnce()
{

  sub = n.subscribe("cmd_vel", 1000, cmd_vel_callback);
  ros::Publisher pub_lmotor = n.advertise<std_msgs::Float64>("lw_vtarget", 1000);
  ros::Publisher pub_rmotor = n.advertise<std_msgs::Float64>("rw_vtarget", 1000);

  rw_vel = 1.0 * dx + dr * dist_lrw / 2;
  lw_vel = 1.0 * dx - dr * dist_lrw / 2;

  //std_msgs::Float64 rw_vel_cast = rw_vel;
  //std_msgs::Float64 lw_vel_cast = lw_vel;
  //ROS_INFO("rw lw: [%d] [%d]", rw, lw);
  pub_lmotor.publish(rw_vel);
  pub_rmotor.publish(lw_vel);

  ++ticks_since_target; 

}// spinOnce


/**
* Loop which call spinOnce
*/
int main(int argc, char **argv)
{
  init();
  ros::Rate loop_rate(rate);
  ros::Rate idle(10);
  tick_since_target = timeout_ticks;

  while(ros::ok())
  {
    while((ros::ok()) && (ticks_since_target < timeout_ticks))
    {
      spinOnce();
      loop_rate.sleep();
      idle.sleep();
    }// while(&&)
  }// while()      
}// main

  
