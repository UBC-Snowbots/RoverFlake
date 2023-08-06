/*
 * Created By: Tate Kolton and Ihsan Olawale
 * Created On: July 4, 2022
 * Description: Header file for recieving messages from pro controller and
 * relaying them to arm hardware driver module
 */

#ifndef ARM_HARDWARE_DRIVER_MYNODE_H
#define ARM_HARDWARE_DRIVER_MYNODE_H

// Component ID's
#define ELEC_BOX_ID 100

//rover standard
#define INIT_STATUS -10
#define OFFLINE 0
#define STANDBY 5
#define ONLINE 1
#define ERROR -1


// STD Includes
#include <iostream>
#include <string>
#include <cstdio>
#include <unistd.h>

// ROS Includes
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
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

class GimbalComm {
  public:
    GimbalComm(ros::NodeHandle& nh);
    void sendMsg(std::string outMsg);
    void recieveMsg();
    void pubUtility();
    
    //new serial
    unsigned long baud = 115200;
    string port = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0";//"/dev/usb/hiddev0"; //will be best if we sift through



    // character representations of buttons for arm communication
    const char T1 = '1';
    const char T2 = '2';
    // const char J3 = '3';
    // const char J4 = '4';
    // const char J5 = '5';
    // const char J6 = '6';

  private:
    ros::NodeHandle nh;

  
    ros::Publisher pos_pub;
    ros::Publisher vitals_pub;

    serial::Serial mcu_board;

  struct utility_component{
    int id;
    _Float64 temperature;
    _Float64 battery_voltage;
  };

  struct vital{
    std_msgs::Int16 status;
  };
  
  utility_component elec_box;


};
#endif // ARM_HARDWARE_DRIVER_MYNODE_H
