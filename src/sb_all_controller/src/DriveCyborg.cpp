/*
 * Created By: Rowan Zawadkzki, modified from the old procontroller_snowbots
 * Created On: December 21st, 2019
 * Description: Uses Libevdev to turn Nintendo Switch Pro Controller left
 * joystick inputs into a ROS Twist message
 * old summary^^
 */

#include "../include/DriveCyborg.h"

int32_t button[NUM_BUTTONS];
int32_t Tbutton[NUM_BUTTONS];

_Float32 axes[8];
_Float32 Taxes[8];
bool proccessing;
bool switchMode;

// vital vitals;

// Read the master documentation if there's any issues with this package
DriveController::DriveController(int argc, char **argv, string node_name)
{
    string joyTopic = "/joy";
    //string state_topic = "/"
    // string moveGrpPublisher = "/move_group_trigger";
    ros::init(argc, argv, node_name);
    ros::NodeHandle private_nh("~");
   
    pubmove = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 55);
    //statuspub = private_nh.advertise<std_msgs::Int16>();
    joyinput = private_nh.subscribe(joyTopic, 55, &DriveController::joyCallback, this);
    // pubmovegrp = private_nh.advertise<std_msgs::Bool>(moveGrpPublisher,1);
    system("toilet DRIVE CONTROL ONLINE");



}

void DriveController::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    // int32_t A = msg->buttons[0];
    // int32_t B = msg->buttons[1];
    // int32_t X = msg->buttons[2];
    // int32_t Y = msg->buttons[2];
    // ROS_INFO("reacieved msg");
    // ROS_INFO("A is %i", msg->buttons[0]);
    // ROS_INFO("B is %i", msg->buttons[1]);
    // ROS_INFO("X is %i", msg->buttons[2]);
    // ROS_INFO("Y is %i", msg->buttons[3]);
    // ROS_INFO("HOME is %f", msg->axes[0]);
   max_speed = (msg->axes[2] + 1)/2;
   x = msg->axes[1];
   z = msg->axes[3];
    

    //}
    // proccessing = true;
    // printState();
    publishVelocity();
}

void DriveController::publishVelocity()
{
    geometry_msgs::Twist msg;

    msg.linear.x = -(x * max_speed);
    msg.angular.z = z * max_speed;
    pubmove.publish(msg);
}

