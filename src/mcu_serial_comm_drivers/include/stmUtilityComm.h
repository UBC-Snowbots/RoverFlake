/*
 * Created By: Tate Kolton and Ihsan Olawale
 * Created On: July 4, 2022
 * Description: Header file for recieving messages from pro controller and
 * relaying them to arm hardware driver module
 */

#ifndef ARM_HARDWARE_DRIVER_MYNODE_H
#define ARM_HARDWARE_DRIVER_MYNODE_H

// STD Includes
#include <iostream>
#include <string>
#include <cstdio>
#include <unistd.h>

// ROS Includes
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>

// Snowbots Includes
#include <sb_msgs/ArmPosition.h>
// #include <sb_utils.h> //from snowflake

// Other
#include <serial/serial.h>


using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

class STMComm {
  public:
    STMComm(ros::NodeHandle& nh);
    void sendMsg(std::string outMsg);
    void recieveMsg();
    void pubUtility();
    
    //new serial
    unsigned long baud = 115200;
    string port = "/dev/ttyUSB0"; //will be best if we sift through



    // character representations of buttons for arm communication
    const char T1 = '1';
    const char T2 = '2';
    // const char J3 = '3';
    // const char J4 = '4';
    // const char J5 = '5';
    // const char J6 = '6';

  private:
    ros::NodeHandle nh;

  
    ros::Publisher pubUtilityData;

    serial::Serial stm_board;
};
#endif // ARM_HARDWARE_DRIVER_MYNODE_H
