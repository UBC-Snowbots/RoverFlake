/*
 * Created By: Rowan Zawadzki/
 * Created On: July 4, 2022
 * Description: This package receives info from / topic 
 * serial data via callback to be recieved by the utility MCU (stm)
 */

#include "../include/stmUtilityComm.h"

STMComm::STMComm(ros::NodeHandle& nh) : nh(nh) {
    // Setup NodeHandles

    ros::NodeHandle private_nh("~");

    // Setup Subscribers
    int queue_size = 55;

   // subPro = nh.subscribe( 
   //     "/cmd_arm", queue_size, &ArmHardwareDriver::allControllerCallback, this);

   pubUtilityData = private_nh.advertise<std_msgs::Float64>("/rover_utility/elec_box_temperature", 1);

try{
    stm_board.setBaudrate(baud);
    stm_board.setPort(port);
    stm_board.open();
    stm_board.setDTR(false);
    stm_board.setRTS(false);
    ROS_INFO("Serial Port Opened");
} catch(serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port");
       // return -1;
elec_box.id = ELEC_BOX_ID;
elec_box.temperature = -99.99;
}
    while(ros::ok()){
        if(stm_board.available() > 0){
            recieveMsg();
        }
       // ROS_WARN("No data available");

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
    //int timeoutCounter = 0; //serial library should have this functionality already
    
   // std::string read_line = stm_board.readline(9000, "\n"); //another way to do it
    do {
        next_char = stm_board.read();
        buffer += next_char;

    //    if(timeoutCounter > 50){
    //     ROS_INFO("timed out");
    //     next_char = "Z";
    //    }
    } while (next_char != "\n");
       std::size_t found = buffer.find("Temperature = ");
      
   if (found != std::string::npos) {
                // Extract the temperature value
                std::string temp_string = buffer.substr(found + 14); // 14 is the length of "Temperature = "

                // Convert the temperature value to a float
                _Float32 temp = std::stof(temp_string);
                elec_box.temperature = temp;
                ROS_INFO_STREAM("Temperature: " << elec_box.temperature << " C");

            }
    // ROS_INFO("Tempurature Buffer: %s", buffer.c_str());

    pubUtility();
    // // Update parameters based on feedback


}


void STMComm::pubUtility(){
std_msgs::Float64 msg;
msg.data = elec_box.temperature;
pubUtilityData.publish(msg);
}


