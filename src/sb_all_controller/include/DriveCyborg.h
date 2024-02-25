/*
 * Created By: Kevin Lin
 * Created On: December 21st, 2019
 * Description: Simple header file for switch controller-->ROS Twist message cpp
 */

#ifndef ALLCONTROLLER_SNOWBOTS_CONTROLLER_H
#define ALLCONTROLLER_SNOWBOTS_CONTROLLER_H
#define NUM_BUTTONS 12

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <std_msgs/Int16.h>
//#include <libevdev-1.0/libevdev/libevdev.h>
#include <ros/ros.h>
#include <sys/fcntl.h>
#include <tuple>
#include <unistd.h>

using namespace std;

class DriveController {
  public:
    DriveController(int argc, char** argv, std::string node_name);

  private: //TODO: make vel relative to analog values
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

    //tuple<double, double> publishMoveXZ(double x_new, double z_new, double x_old, double z_old);
    void publishVelocity();

    void publishStatus();

    // see documentation to changes sensitivities at runtime... what documentation?

    double x;
    double z;

   
    ros::Publisher pubmove;
   
    ros::Subscriber joyinput;

    float max_speed = 0.5; //1 = max
    float max_speed_ang = max_speed / 2;


  // struct vital{
  //   int controller_status;
  // };
};

#endif // ALLCONTROLLER_SNOWBOTS_CONTROLLER_H
