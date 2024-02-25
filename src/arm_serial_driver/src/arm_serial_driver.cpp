/*
 * Created By: Rowan Zawadzki
 * Created On: Aug 20, 2023
 * Description: Node for sending msgs to arm, for the new zephyr firmware babyyyy
 */

#include "../include/armSerialDriver.h"
#include <ros/callback_queue.h>
#include <thread>



void armReceiverThread(ArmSerialDriver& zephyrComm) {
    ros::Rate loop_rate(100);  // specify the rate in Hz, adjust as needed
     while(!zephyrComm.teensy.isOpen()){
        //wait to make sure teensy is open
    }
    ROS_WARN("Teensy Open");

    while (ros::ok()) {
       //if(zephyrComm.teensy.available() > 0){
 
      //  ROS_WARN("Check Read");

//    while(!zephyrComm.teensy.available()){

//    }

    zephyrComm.recieveMsg();
loop_rate.sleep();
    //zephyrComm.teensy.flushInput();

 
    }
        //if ros dies, close serial port

    zephyrComm.teensy.close();

    ROS_ERROR("Teensy RX Major Error");
    ROS_ERROR("Teensy RX Major Error");
    ROS_ERROR("Teensy RX Major Error");


}

int main(int argc, char** argv) {


    // Setup your ROS node
    std::string node_name = "arm_serial_driver";
    ros::CallbackQueue arm_dedicated_queue;
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    nh.setCallbackQueue(&arm_dedicated_queue);

    // Create an instance of your class
    ArmSerialDriver zephyrComm(nh);

   // Create a separate thread for uart recieving loop
    std::thread arm_thread(armReceiverThread, std::ref(zephyrComm));
   // arm_thread.join();

    // Start up ros. This will continue to run until the node is killed
    ros::MultiThreadedSpinner spinner(0); //0 means use all threads
    

    spinner.spin(&arm_dedicated_queue);





    // Once the node stops, return 0
    return 0;
}


