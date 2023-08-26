/*
 * Created By: Tate Kolton and Ihsan Olawale
 * Created On: July 4, 2022
 * Description: Header file for recieving messages from pro controller and
 * relaying them to arm hardware driver module
 */

#ifndef ARM_SERIAL_DRIVER_MYNODE_H
#define ARM_SERIAL_DRIVER_MYNODE_H
static_assert(sizeof(float) == 4, "float is not 32-bit on this platform");
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
#include <std_msgs/Int16.h>

// Snowbots Includes
#include <sb_msgs/ArmPosition.h>
#include <sb_utils.h>


// Other
#include <serial/serial.h>

#define NUM_JOINTS 6

#define TX_UART_BUFF 128
#define RX_UART_BUFF 128


using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

class ArmSerialDriver {
  public:
    ArmSerialDriver(ros::NodeHandle& nh);

    void armPositionCmdCallBack(const sb_msgs::ArmPosition::ConstPtr& cmd_msg);


    //new serial
    unsigned long baud = 115200;
    string port = "/dev/serial/by-id/usb-ZEPHYR_UBC_ROVER_Arm_500100C6224069D7-if00";


  
    const char jointMode = 'j';

    double ppr      = 400.0;
    double encppr   = 512.0;

    bool homeFlag = false;
    const char mode = jointMode;

    // hardware interface communication variables
    std::vector<int> encPos, encCmd;
    std::vector<double> armCmd, armPos, poseCmd, encStepsPerDeg;
    std::vector<double> reductions{50.0, 160.0, 92.3077, 43.936, 57, 14};

  private:
    ros::NodeHandle nh;
    //void poseSelectCallback(const sb_msgs::ArmPosition::ConstPtr& poseAngles);
    void sendMsg(uint8_t outMsg[TX_UART_BUFF]);
    void recieveMsg();
     void parseArmAngleUart(std::string msg);

    bool homed = false;

   
    ros::Subscriber subPro;
    ros::Subscriber subPose;
    ros::Subscriber subCmdPos;
    ros::Publisher pubCurrPos;

    serial::Serial teensy;

    struct Axis{
      float angle_pos;
      float des_angle_pos;
    };
    Axis axes[NUM_JOINTS];

    struct vital{
      ros::Publisher pubber;
      std_msgs::Int16 statusmsg;
      void pub(int status){
        statusmsg.data = status;
        pubber.publish(statusmsg);
      }
    };

  

};
#endif // ARM_HARDWARE_SERIAL_MYNODE_H
