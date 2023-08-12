/*
 * Created By: Rowan Zawadzki/
 * Created On: July 4, 2022
 * Description: This package receives info from / topic 
 * serial data via callback to be recieved by the utility MCU (stm)
 */

#include "../include/GimbalComm.h"

GimbalComm::GimbalComm(ros::NodeHandle& nh) : nh(nh) {
    // Setup NodeHandles
    ros::NodeHandle private_nh("~");

    

    // Setup Subscribers
    int queue_size = 5;

   // subPro = nh.subscribe( 
   //     "/cmd_arm", queue_size, &ArmHardwareDriver::allControllerCallback, this);

    //Setup Publishers
    vitals_pub = private_nh.advertise<std_msgs::Int16>("/status/net_ags", 5);
    pos_pub = private_nh.advertise<std_msgs::Float64>("/net_ags/pos", 5);
    pos_sub = private_nh.subscribe("/net_ags/cmd", 5, &GimbalComm::netags_callback, this);

    //Setup Vitals
    vital vitals;
    vitals.status.data = INIT_STATUS;

try{
    mcu_board.setBaudrate(baud);
    mcu_board.setPort(port);
    mcu_board.open();
    mcu_board.setDTR(false);
    mcu_board.setRTS(false);
    ROS_INFO("Serial Port Opened");
} catch(serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port");
       // return -1;
elec_box.id = ELEC_BOX_ID;
elec_box.temperature = -99.99;
}
    while(ros::ok()){
        if(mcu_board.available() > 0){
            recieveMsg();
        }
       // ROS_WARN("No data available");

    }


}

// Serial Implementation 
void GimbalComm::sendMsg(std::string outMsg) {
    // Send everything in outMsg through serial port
    //ROS_INFO("attempting send");
    mcu_board.write(outMsg);
    ROS_INFO("Requested: %s", outMsg.c_str());
}

void GimbalComm::netags_callback(const std_msgs::Float64::ConstPtr &cmdmsg){
    
    sendMsg(std::to_string(cmdmsg->data));

}

void GimbalComm::recieveMsg() {

    std::string next_char = "";
    std::string buffer = "";
    //int timeoutCounter = 0; //serial library should have this functionality already
    
   // std::string read_line = stm_board.readline(9000, "\n"); //another way to do it
    do {
        next_char = mcu_board.read();
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


void GimbalComm::pubUtility(){
std_msgs::Float64 msg;
msg.data = elec_box.temperature;
pos_pub.publish(msg);
}


