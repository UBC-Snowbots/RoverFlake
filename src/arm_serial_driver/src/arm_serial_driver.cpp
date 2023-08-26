/*
 * Created By: Rowan Zawadzki
 * Created On: Aug 20, 2023
 * Description: Node for sending msgs to arm, for the new zephyr firmware babyyyy
 */

#include "../include/armSerialDriver.h"
#include <ros/callback_queue.h>

int main(int argc, char** argv) {


    // Setup your ROS node
    std::string node_name = "arm_serial_driver";
    ros::CallbackQueue arm_dedicated_queue;
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    nh.setCallbackQueue(&arm_dedicated_queue);

    // Create an instance of your class
    ArmSerialDriver zephyrComm(nh);

    // Start up ros. This will continue to run until the node is killed

    ros::MultiThreadedSpinner spinner(8); //0 means use all threads
    spinner.spin(&arm_dedicated_queue);

    // Once the node stops, return 0
    return 0;
}
