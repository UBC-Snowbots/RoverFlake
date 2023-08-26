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
    int queue_size = 55;

    vital vitals;

    vitals.pubber = private_nh.advertise<std_msgs::Int16>("/status/arm_mcu", 5);
    vitals.pub(OFFLINE);

    sleep(1);
    vitals.pub(INIT_STATUS);

    subCmdPos = nh.subscribe(
        "/cmd_pos_arm", queue_size, &ArmSerialDriver::armPositionCmdCallBack, this);

   // subPose = private_nh.subscribe("/cmd_pose", 1, &ArmSerialDriver::poseSelectCallback, this);

    pubObservedPos = private_nh.advertise<sb_msgs::ArmPosition>("/observed_pos_arm", 1);
    sleep(1);
    vitals.pub(STANDBY);
    teensy.setBaudrate(baud);
    teensy.setPort(port);
    teensy.open();
    teensy.setDTR(false);
    teensy.setRTS(false);

    sleep(1);

    encCmd.resize(num_joints_);
    armCmd.resize(num_joints_);
    encStepsPerDeg.resize(num_joints_);
    armPos.resize(num_joints_);
    encPos.resize(num_joints_);
    armCmd.resize(num_joints_);
    poseCmd.resize(num_joints_);

    for (int i = 0; i < num_joints_; i++) {
        encStepsPerDeg[i] = reductions[i] * ppr * 5.12 / 360.0;
    }
    
    vitals.pub(ONLINE);

    //ros::spinOnce;
    sleep(0.5);

}

    void ArmSerialDriver::armPositionCmdCallBack(const sb_msgs::ArmPosition::ConstPtr& cmd_msg){

}


// void ArmSerialDriver::poseSelectCallback(
// const sb_msgs::ArmPosition::ConstPtr& poseAngles) {
//     poseCmd.assign(poseAngles->positions.begin(), poseAngles->positions.end());
//     jointPosToEncSteps(poseCmd, encCmd);
    
//     std::string outMsg = "PM";
//     for(int i=0; i < num_joints_; i++) {
//         outMsg += 'A' + i;
//         outMsg += std::to_string(encCmd[i]);
//     }

//     outMsg += "/n";
//     sendMsg(outMsg);
//     recieveMsg();
// }

// void ArmSerialDriver::armPositionCallBack(
// const sb_msgs::ArmPosition::ConstPtr& commanded_msg) {
    
//     armCmd.assign(commanded_msg->positions.begin(),
//                   commanded_msg->positions.end());
//     jointPosToEncSteps(armCmd, encCmd);

//     std::string outMsg = "MT";
//     for (int i = 0; i < num_joints_; ++i) {
//         outMsg += 'A' + i;
//         outMsg += std::to_string(encCmd[i]);
//     }
//     outMsg += "\n";
//     sendMsg(outMsg);
//     recieveMsg();
// }

// void ArmSerialDriver::updateEncoderSteps(std::string msg) {
//     size_t idx1 = msg.find("A", 2) + 1;
//     size_t idx2 = msg.find("B", 2) + 1;
//     size_t idx3 = msg.find("C", 2) + 1;
//     size_t idx4 = msg.find("D", 2) + 1;
//     size_t idx5 = msg.find("E", 2) + 1;
//     size_t idx6 = msg.find("F", 2) + 1;
//     size_t idx7 = msg.find("Z", 2) + 1;
//     encPos[0]   = std::stoi(msg.substr(idx1, idx2 - idx1));
//     encPos[1]   = std::stoi(msg.substr(idx2, idx3 - idx2));
//     encPos[2]   = std::stoi(msg.substr(idx3, idx4 - idx3));
//     encPos[3]   = std::stoi(msg.substr(idx4, idx5 - idx4));
//     encPos[4]   = std::stoi(msg.substr(idx5, idx6 - idx5));
//     encPos[5]   = std::stoi(msg.substr(idx6, idx7 - idx6));
// }

// void ArmSerialDriver::encStepsToJointPos(
// std::vector<int>& enc_steps, std::vector<double>& joint_positions) {
//     for (int i = 0; i < enc_steps.size(); ++i) {
//         // convert enc steps to joint deg
//         joint_positions[i] =
//         static_cast<double>(enc_steps[i]) / encStepsPerDeg[i];
//     }
// }

// void ArmSerialDriver::jointPosToEncSteps(std::vector<double>& joint_positions,
//                                            std::vector<int>& enc_steps) {
//     for (int i = 0; i < joint_positions.size(); ++i) {
//         // convert joint deg to enc steps
//         enc_steps[i] = static_cast<int>(joint_positions[i] * encStepsPerDeg[i]);
//     }
// }

// Libserial Implementation

void ArmSerialDriver::sendMsg(std::string outMsg) {
    // Send everything in outMsg through serial port
    //ROS_INFO("attempting send");
    teensy.write(outMsg);
    ROS_INFO("Sent via serial: %s", outMsg.c_str());
}

// void ArmSerialDriver::recieveMsg() {

//     std::string next_char = "";
//     std::string buffer = "";
//     int timeoutCounter = 0;
//     do {
//         timeoutCounter ++;
//         next_char = teensy.read();
//         buffer += next_char;
//     //    if(timeoutCounter > 50){
//     //     ROS_INFO("timed out");
//     //     next_char = "Z";
//     //    }
//     } while (next_char != "Z");

//      ROS_INFO("buffer: %s", buffer.c_str());


//     // // Update parameters based on feedback
//     updateEncoderSteps(buffer);
//     encStepsToJointPos(encPos, armPos);
//    // updateHWInterface();

// }

