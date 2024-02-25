/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbots UI Tester. This publishes random twist messages to the "/cmd_vel"
 * topic in place of the procontroller package
 */

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include <stdlib.h>
#include <sstream>
#include "std_msgs/Float64.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "ui_tester");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    ros::Publisher pubTemp = n.advertise<std_msgs::Float64>("/rover_utility/elec_box_tempurature", 5);


    ros::Rate rate(2);

    srand(time(0));

    while (ros::ok()) {
        // Declares the message to be sent
        geometry_msgs::Twist msg;
        std_msgs::Float64 temp;


        // Random x value between -2 and 2
        msg.linear.x = 4 * double(rand()) / double(RAND_MAX) - 2;

        temp.data = 60 * double(rand()) / double(RAND_MAX);

        // Random z value between -3 and 3
        msg.angular.z = 6 * double(rand()) / double(RAND_MAX) - 3;

        // Publish the message
        pub.publish(msg);

        pubTemp.publish(temp);

        // Delays until it is time to send another message
        rate.sleep();
    }

    return 0;
}
