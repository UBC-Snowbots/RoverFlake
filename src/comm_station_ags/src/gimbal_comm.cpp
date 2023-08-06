/*
 * Created By: Rowan Zawadzki
 * Created On: July 15, 2023
 * Description: Node for recieving messages from STM tempurature/utility board.
 */

#include "../include/GimbalComm.h"
#include <ros/callback_queue.h>

int main(int argc, char** argv) {


    // Setup your ROS node
    std::string node_name = "stm_utility_communications_node";
    ros::CallbackQueue ros_queue;
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);

    // Create an instance of your class
    GimbalComm GimbalCommInst(nh);

    // Start up ros. This will continue to run until the node is killed

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);

    // Once the node stops, return 0
    return 0;
}
