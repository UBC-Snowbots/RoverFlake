/*
 * Created By: Tate Kolton and Ihsan Olawale
 * Created On: July 4, 2022
 * Description: Header file for recieving messages from pro controller and
 * relaying them to arm hardware driver module
 */

#ifndef ARM_SERIAL_DRIVER_MYNODE_H
#define ARM_SERIAL_DRIVER_MYNODE_H

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



    // character representations of buttons for arm communication
    const char leftJSU     = 'A';
    const char leftJSD     = 'B';
    const char rightJSU    = 'C';
    const char rightJSD    = 'D';
    const char buttonA     = 'E';
    const char buttonB     = 'F';
    const char buttonX     = 'G';
    const char buttonY     = 'H';
    const char triggerL    = 'I';
    const char triggerR    = 'J';
    const char bumperL     = 'K';
    const char bumperR     = 'L';
    const char buttonARel  = 'M';
    const char buttonBRel  = 'N';
    const char buttonXRel  = 'O';
    const char buttonYRel  = 'P';
    const char triggerLRel = 'Q';
    const char triggerRRel = 'R';
    const char bumperLRel  = 'S';
    const char bumperRRel  = 'T';
    const char arrowL      = 'U';
    const char arrowR      = 'V';
    const char arrowU      = 'W';
    const char arrowD      = 'X';
    const char arrowRLRel  = '0';
    const char leftJSRel   = 'Y';
    const char rightJSRel  = 'Z';
    const char rightJSPress = '7';
    const char rightJSPressRel = '8';
    const char homeVal     = '4';
    const char homeValEE = '6';

  
    const char jointMode = 'j';

    int num_joints_ = 6;
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
    void sendMsg(std::string outMsg);
   
    ros::Subscriber subPro;
    ros::Subscriber subPose;
    ros::Subscriber subCmdPos;
    ros::Publisher pubObservedPos;

    serial::Serial teensy;

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
