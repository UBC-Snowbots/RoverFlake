/*
 * Created By: Tate Kolton
 * Created On: July 4, 2022
 * Description: This package receives info from /cmd_arm topic and publishes
 * serial data via callback to be recieved by the arm MCU (Teensy 4.1)
 */

#include "../include/stmUtilityComm.h"

STMComm::STMComm(ros::NodeHandle& nh) : nh(nh) {
    // Setup NodeHandles

    ros::NodeHandle private_nh("~");

    // Setup Subscribers
    int queue_size = 55;

   // subPro = nh.subscribe( 
   //     "/cmd_arm", queue_size, &ArmHardwareDriver::allControllerCallback, this);

   pubUtilityData = private_nh.advertise<std_msgs::String>("/utility_board_data", 1);


    stm_board.setBaudrate(baud);
    stm_board.setPort(port);
    stm_board.open();


    while(ros::ok()){
        if(stm_board.available() > 0){
            recieveMsg();
            ROS_INFO("START READ");

        }
                    ROS_INFO("WAIT");

    }


}

// Serial Implementation 
void STMComm::sendMsg(std::string outMsg) {
    // Send everything in outMsg through serial port
    //ROS_INFO("attempting send");
    stm_board.write(outMsg);
    ROS_INFO("Sent via serial: %s", outMsg.c_str());
}

void STMComm::recieveMsg() {

    std::string next_char = "";
    std::string buffer = "";
    int timeoutCounter = 0;
    do {
        timeoutCounter ++;
        next_char = stm_board.read();
        buffer += next_char;
    //    if(timeoutCounter > 50){
    //     ROS_INFO("timed out");
    //     next_char = "Z";
    //    }
    ROS_INFO("READING");
    } while (next_char != "\n");

     ROS_INFO("Tempurature Buffer: %s", buffer.c_str());

    pubUtility();
    // // Update parameters based on feedback


}


void STMComm::pubUtility(){

}


