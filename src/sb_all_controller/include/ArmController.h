/*
 * Created By: Rowan Zawadzki
 * Created On: Aug 25th, 2023
 * Description: Drives arm via xbox controller and arm serial driver
 */

//make sure file is included once
#ifndef ARMCONTROLLER_SNOWBOTS_CONTROLLER_H
#define ARMCONTROLLER_SNOWBOTS_CONTROLLER_H

//queue sizes
#define PUB_ARM_DES_POS_ANGLE_QUEUE_SIZE 10
#define JOY_ARM_QUEUE_SIZE 10
#define SUB_ARM_POS_QUEUE_SIZE 10


#define NUM_JOY_BUTTONS 11
#define NUM_JOY_AXES 8

#define NUM_ARM_AXES 6

//joy axes inputs to arm axes, diagram is in my notebook
#define JOY_AXIS_1_CW_INDEX 5
#define JOY_AXIS_1_CCW_INDEX 2 

#define JOY_AXIS_2_INDEX 7 
#define JOY_AXIS_3_INDEX 1 
#define JOY_AXIS_4_INDEX 0 
#define JOY_AXIS_5_INDEX 4 
#define JOY_AXIS_6_INDEX 3 

#define SPEED_SCALE_JOINT_FACTOR 80


#define HOME_XBOX_INDEX 7



#include <cstdio>
#include <cstdlib>
#include <cstring>
// #include <geometry_msgs/Twist.h>
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

#include <sb_msgs/ArmPosition.h>





using namespace std;

class ArmController {
  public:
    ArmController(int argc, char** argv, std::string node_name);

  private: //TODO: make vel relative to analog values
    void setup();
    void processInputs();
    void readJoyInputs(const sensor_msgs::Joy::ConstPtr& msg);
    void readArmPosition(const sb_msgs::ArmPosition::ConstPtr& msg);
    void publishCmds();

    //tuple<double, double> publishMoveXZ(double x_new, double z_new, double x_old, double z_old);

   

  
    std::string armOutMsg, armOutVal;
// character representations of buttons for arm communication

    // arm modes
    const char jointMode = '1';
    const char IKMode = '2';
    const char drillMode = '3';


    struct libevdev* dev = NULL;
    enum Mode { wheels = 0, arm_joint_space = 1, arm_cartesian = 2, drilling = 3, num_modes = 2 };
    Mode state;
    bool debug = false;
    ros::Publisher pubArmDesPosAngle;
    ros::Publisher statuspub;
    ros::Subscriber joyinput;
    ros::Subscriber subArmPos;


    sb_msgs::ArmPosition current_arm;
    sb_msgs::ArmPosition des_arm;

  //int speed[NUM_ARM_AXES] = {50, 50, 50, 50, 50, 50}; // Initialize all elements to 50
  float max_speed[NUM_ARM_AXES] = {70.0, 6.0, 60.0, 100.0, 40.0, 60.0}; // Initialize all elements to 0.5
 // float max_speed_ang[NUM_ARM_AXES] = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25}; // Initialize all elements to 0.25
  // Initialize current_speed array, assuming you want to initialize all to 0.0 for starters
  float current_speed[NUM_ARM_AXES] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  //float max_increment_angle[NUM_ARM_AXES] = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  int max_increment_steps[NUM_ARM_AXES] = {80, 80, 80, 60, 60, 15};

  bool triggers_init = false; //xbox triggers from ros_joy start at 0.0, when they are spring loaded to 1.0
  int axis_dirs[NUM_ARM_AXES] = {1, 1, 1, -1, 1, 1};
  sensor_msgs::Joy xbox;


  // struct vital{
  //   int controller_status;
  // };
};

#endif // ALLCONTROLLER_SNOWBOTS_CONTROLLER_H
