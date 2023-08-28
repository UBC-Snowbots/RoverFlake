/*
 * Created By: Tate Kolton
 * Created On: July 4, 2022
 * Description: This package receives info from /cmd_arm topic and publishes
 * serial data via callback to be recieved by the arm MCU (Teensy 4.1)
 */

#include "../include/armSerialDriver.h"

ArmSerialDriver::ArmSerialDriver(ros::NodeHandle& nh) : nh(nh) {
    // Setup NodeHandles

    ros::NodeHandle private_nh("~");

    // Setup Subscribers
    int queue_size = 10;

    vital vitals;

    vitals.pubber = private_nh.advertise<std_msgs::Int16>("/status/arm_mcu", 5);
    vitals.pub(OFFLINE);

    sleep(1);
    vitals.pub(INIT_STATUS);

    subCmdPos = nh.subscribe(
        "/arm/cmd_pos_angle", queue_size, &ArmSerialDriver::armPositionCmdCallBack, this);

   // subPose = private_nh.subscribe("/cmd_pose", 1, &ArmSerialDriver::poseSelectCallback, this);

    pubCurrPos = private_nh.advertise<sb_msgs::ArmPosition>("/arm/curr_pos_angle", 10);
    sleep(1);
    vitals.pub(STANDBY);
    teensy.setBaudrate(baud);
    teensy.setPort(port);
    //teensy.setTimeout(timeout_uart);
 
    teensy.open();
    teensy.setDTR(true);
    teensy.setRTS(false);

    sleep(1);

    encCmd.resize(NUM_JOINTS);
    armCmd.resize(NUM_JOINTS);
    encStepsPerDeg.resize(NUM_JOINTS);
    armPos.resize(NUM_JOINTS);
    encPos.resize(NUM_JOINTS);
    armCmd.resize(NUM_JOINTS);
    poseCmd.resize(NUM_JOINTS);

    for (int i = 0; i < NUM_JOINTS; i++) {
        encStepsPerDeg[i] = reductions[i] * ppr * 5.12 / 360.0;
    }
    
    vitals.pub(ONLINE);

    //ros::spinOnce;
    // sleep(0.5);
    // ros::Rate loop_rate(50); // 100 Hz
    // //ROS_INFO("Arm Online");
    // while(ros::ok()){
    
  
    // loop_rate.sleep();
  
    // }

}

    void ArmSerialDriver::armPositionCmdCallBack(const sb_msgs::ArmPosition::ConstPtr& cmd_msg){

            uint8_t tx_msg[TX_UART_BUFF];
            char tx_temp_msg[TX_UART_BUFF];


            if(cmd_msg->home_cmd){
            sendMsg((uint8_t *)"$h()\n");
            ROS_INFO("sent home msg\n");            
            }else if (homed){
            sprintf(tx_temp_msg, "$i(%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f)\n", cmd_msg->positions[0], cmd_msg->positions[1], cmd_msg->positions[2], cmd_msg->positions[3], cmd_msg->positions[4], cmd_msg->positions[5]);
            std::memcpy(tx_msg, tx_temp_msg, sizeof(tx_temp_msg));  // +1 to copy the null-terminator
            sendMsg(tx_msg);
            //fresh_rx_angle = false;
 
             } //else if(!arm_inited){
            //     //init arm
            //     sendMsg((uint8_t *)"init\n\r\0");
            //     arm_inited = true;
            // }

            
            

            //sendMsg(tx_msg);

            
}

void ArmSerialDriver::sendMsg(uint8_t outMsg[TX_UART_BUFF]) {
    // Send everything in outMsg through serial port
    //ROS_INFO("attempting send");
    std::string str_outMsg(reinterpret_cast<const char*>(outMsg), TX_UART_BUFF);
    //std::to_string(str_outMsg);  // +1 to copy the null-terminator

    teensy.write(str_outMsg);
    ROS_ERROR("Sent via serial: %s", str_outMsg.c_str());
    teensy.flushOutput();
}

void ArmSerialDriver::recieveMsg() {

    std::string next_char = "";
    std::string buffer = "";
    int timeoutCounter = 0;
    if (teensy.available() > 0){
        //timeoutCounter ++;
     //   next_char = teensy.read(); 
        buffer = teensy.readline();

        // if(next_char == "\n" || next_char == "\r" || next_char == "\0"){
        //     timeoutCounter = RX_UART_BUFF;
        // }

      
    }
   teensy.flushInput();

if (buffer.size() > 0){
        ROS_WARN("%s", buffer.c_str());
     if(buffer.find("my_angleP") != std::string::npos){
        parseArmAngleUart(buffer);
     }
    if(buffer.find("Arm Ready") != std::string::npos){
        homed = true;
       // fresh_rx_angle = true;
     }




}
}

 void ArmSerialDriver::parseArmAngleUart(std::string msg){
     //ROS_INFO("Parsing Angle buffer: %s", msg.c_str());
          sb_msgs::ArmPosition angle_echo;
          angle_echo.positions.resize(NUM_JOINTS);
    
	if (sscanf(msg.c_str(), "$my_angleP(%f, %f, %f, %f, %f, %f)\n",  &axes[0].angle_pos, &axes[1].angle_pos, &axes[2].angle_pos, &axes[3].angle_pos, &axes[4].angle_pos, &axes[5].angle_pos) == 6)
	{
		// All axes angles are in axes[i].des_angle_pos
		ROS_INFO("Absolute Angle Position Echo Accepted:");
        for(int i = 0; i < NUM_JOINTS; i++){
        angle_echo.positions[i] = axes[i].angle_pos;

        }
        pubCurrPos.publish(angle_echo);
        ROS_INFO("Absolute Angle Position Echo Update Successfull");
        fresh_rx_angle = true;

	}
	else
	{
		// Error handling: could not parse all 6 angles, or message is messed up.
		ROS_ERROR("Absolute Angle Position Echo Rejected, incorrect syntax");
       // fresh_rx_angle = false;

		return;
	}


 }


