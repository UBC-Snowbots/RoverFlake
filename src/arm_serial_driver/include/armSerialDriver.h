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

#define AXIS_1_DIR 1
#define AXIS_2_DIR 1
#define AXIS_3_DIR 1
#define AXIS_4_DIR 1
#define AXIS_5_DIR 1
#define AXIS_6_DIR 1


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
    unsigned long baud = 19200;
    string port = "/dev/serial/by-id/usb-ZEPHYR_UBC_ROVER_Arm_500100C6224069D7-if00";

    serial::Serial teensy;
    serial::Timeout timeout_uart = serial::Timeout::simpleTimeout(1000); // E.g., 1000 ms or 1 second
    bool TEST_DANCE = 1;

  
    const char jointMode = 'j';

    double ppr      = 400.0;
    double encppr   = 512.0;

    bool homeFlag = false;
    bool fresh_rx_angle = false;

    const char mode = jointMode;

    // hardware interface communication variables
    std::vector<int> encPos, encCmd;
    std::vector<double> armCmd, armPos, poseCmd, encStepsPerDeg;
    std::vector<double> reductions{50.0, 160.0, 92.3077, 43.936, 57.0, 5.18};
    void recieveMsg();

  private:
    ros::NodeHandle nh;
    //void poseSelectCallback(const sb_msgs::ArmPosition::ConstPtr& poseAngles);
    void sendMsg(std::string outMsg);
    void parseArmAngleUart(std::string msg);


    bool homed = false;
    bool arm_inited = false;


   
    ros::Subscriber subPro;
    ros::Subscriber subPose;
    ros::Subscriber subCmdPos;
    ros::Publisher pubCurrPos;


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




//$P(90.0, 25.0, 40.0, 80.0, 0.0)