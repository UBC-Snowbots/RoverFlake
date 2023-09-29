/*
 * Created By: Rowan Zawadkzki, modified from the old procontroller_snowbots
 * Created On: December 21st, 2019
 * Description: Uses Libevdev to turn Nintendo Switch Pro Controller left
 * joystick inputs into a ROS Twist message
 * old summary^^
 */

#include "../include/ArmController.h"

bool proccessing;
bool switchMode;

// vital vitals;

// Read the master documentation if there's any issues with this package
ArmController::ArmController(int argc, char **argv, string node_name)
{
    string publisher = "/cmd_vel";
    string armPublisher = "/arm/cmd_pos_angle";
    string armCurrPosTopic = "/arm/curr_pos_angle";
    string modePublisher = "/moveit_toggle";
    string joyTopic = "/arm/joy";
    // string state_topic = "/"
    //  string moveGrpPublisher = "/move_group_trigger";
    ros::init(argc, argv, node_name);
    ros::NodeHandle private_nh("~");

    sleep(1);
    pubArmDesPosAngle = private_nh.advertise<sb_msgs::ArmPosition>(armPublisher, PUB_ARM_DES_POS_ANGLE_QUEUE_SIZE);
    // statuspub = private_nh.advertise<std_msgs::Int16>();
    joyinput = private_nh.subscribe(joyTopic, JOY_ARM_QUEUE_SIZE, &ArmController::readJoyInputs, this);
    // pubmovegrp = private_nh.advertise<std_msgs::Bool>(moveGrpPublisher,1);
    subArmPos = private_nh.subscribe(armCurrPosTopic, SUB_ARM_POS_QUEUE_SIZE, &ArmController::readArmPosition, this);
    setup();

    // if(TEST_DANCE){
    //     ROS_INFO("testing...\n");

    // }else{
    ROS_INFO("Ready to drive arm via xbox...\n");
    // }
}

void ArmController::readJoyInputs(const sensor_msgs::Joy::ConstPtr &msg)
{

    for (int i = 0; i < NUM_JOY_BUTTONS; i++)
    {

        xbox.buttons[i] = msg->buttons[i];
    }
    for (int i = 0; i < NUM_JOY_AXES; i++)
    {

        xbox.axes[i] = msg->axes[i];
    }

    processInputs();
}

void ArmController::readArmPosition(const sb_msgs::ArmPosition::ConstPtr &msg)
{

    for (int i = 0; i < NUM_ARM_AXES; i++)
    {
        current_arm.positions[i] = msg->positions[i];
    }
}

void ArmController::setup()
{
    // system(
    //     "rosrun joy joy_node _deadzone:=0.1 _autorepeat_rate:=20 _coalesce_interval:=0.05");

    // populate some data
    current_arm.positions.resize(NUM_ARM_AXES);
    current_arm.velocities.resize(NUM_ARM_AXES);
    des_arm.positions.resize(NUM_ARM_AXES);
    des_arm.velocities.resize(NUM_ARM_AXES);

    xbox.axes.resize(NUM_JOY_AXES);
    xbox.buttons.resize(NUM_JOY_BUTTONS);

    for (int i = 0; i < NUM_ARM_AXES; i++)
    {
        // current_arm.positions[i] = 000.00;
        des_arm.positions[i] = 000.00;
        des_arm.velocities[i] = 00.00;
    }

    ROS_INFO(
        "ArmCONTROLLER INITIATED, CURRENTLY PARSING FOR XBOX CONTROLLER");
}

void ArmController::processInputs()
{
    // ROS_INFO(
    // "PROCESS INPUT");
    // Assuming joy.axes and current_arm are populated
    // if(fresh_rx_angle){
    // Handle the special case for axis 1 with CW and CCW triggers

    while (!triggers_init)
    {

        if (xbox.axes[JOY_AXIS_1_CW_INDEX] > -0.9 || xbox.axes[JOY_AXIS_1_CCW_INDEX] > -0.9)
        {      
            ROS_WARN("Please hold down and release L and R xbox triggers to initiate inputs, %f, %f", xbox.axes[JOY_AXIS_1_CCW_INDEX], xbox.axes[JOY_AXIS_1_CW_INDEX]);
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        else
        {
            ROS_WARN("Thanks! Inputs Initiated");
            triggers_init = true;
        }
    } // else{

    float joy_val_axis_1 = xbox.axes[JOY_AXIS_1_CCW_INDEX] - xbox.axes[JOY_AXIS_1_CW_INDEX];

    des_arm.velocities[0] = joy_val_axis_1*axis_dirs[0] * max_speed[0];

    //des_arm.positions[0] =  joy_val_axis_1* max_increment_steps[0]; //add a buffer so small movements aren't buggy, but still won't drift


    // Update desired positions for other axes
    int joy_indices[] = {NULL, JOY_AXIS_2_INDEX, JOY_AXIS_3_INDEX, JOY_AXIS_4_INDEX, JOY_AXIS_5_INDEX, JOY_AXIS_6_INDEX};
    
    for (int i = 1; i < NUM_ARM_AXES; i++)
    {
        float joy_val = xbox.axes[joy_indices[i]];
        des_arm.velocities[i] = joy_val*axis_dirs[i] * max_speed[i]; // degrees per sec
      //  des_arm.positions[i] =  joy_val* max_increment_steps[i]; //add a buffer so small movements aren't buggy, but still won't drift

    }

    if (xbox.buttons[HOME_XBOX_INDEX])
    {
        des_arm.home_cmd = 1;
    }
    else
    {
        des_arm.home_cmd = 0;
    }

    publishCmds();
    // }
}

void ArmController::publishCmds()
{

    pubArmDesPosAngle.publish(des_arm);
}
