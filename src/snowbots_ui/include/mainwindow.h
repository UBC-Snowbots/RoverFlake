/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbots UI
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

// ros
//#include "RosIntegration.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/Float64.h"

static geometry_msgs::Twist twist_message_controller;
extern std_msgs::Float64 elec_box_tempurature_msg;
static geometry_msgs::Twist twist_message_left;
static geometry_msgs::Twist twist_message_right;
extern _Float64 elec_box_tempurature;



namespace Ui {
class MainWindow;
}
class MainWindow : public QMainWindow {
    Q_OBJECT
  public:
    explicit MainWindow(QWidget* parent = nullptr);
    virtual ~MainWindow();

    void handleButton1(){
        //ui->Pos1_button->setText("Clicked!");
        
        std_msgs::String msg;
        std::stringstream ss;
        ss << "pos 1 clicked!";
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());

        arm_pos.publish(msg);
        ros::spinOnce();
    } 
    void
    elec_box_temp_callback(const std_msgs::Float64::ConstPtr& msg) {
        elec_box_tempurature = msg->data;
        ROS_INFO("recieved, %f", elec_box_tempurature);
    }

    static void
    twist_left_callback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
        twist_message_left = *twist_msg;
    }

    static void
    twist_right_callback(const geometry_msgs::Twist::ConstPtr& twist_msg) {
        twist_message_right = *twist_msg;
    }

    void twist_subscriber() {
        ros::Rate loop_rate(10);
        
        twist_left_sub = n.subscribe(
        "/integration_node/lwheels_pub_topic", 1000, twist_left_callback);
        twist_right_sub = n.subscribe(
        "/integration_node/rwheels_pub_topic", 1000, twist_right_callback);
        elec_box_tempurature_sub = n.subscribe<std_msgs::Float64>("/rover_utility/elec_box_tempurature", 10, boost::bind(&MainWindow::elec_box_temp_callback, this, _1));
        // sub = n->subscribe("chatter", 1000, chatterCallback);

        loop_rate.sleep();
        ros::spinOnce();
    }


  public Q_SLOTS:
    void twist_values();
    //void handleButton1();
    void handleButton2();
    void handleButton3();
    void handleButton4();
    void handleButton5();
    void handleButton6();

    /*/private slots: Note if you want to create function from mainwindow.ui
     * delete
     * private slots and put them with public QSLOTS
     * */

  private:
    Ui::MainWindow* ui;
    // RosIntegration* ros_f;
    QTimer* timer;
    ros::NodeHandle n;

    //ros::Rate loop_rate(10);

    ros::Subscriber elec_box_tempurature_sub;

    // twist topics

    ros::Subscriber twist_controller_sub;
    ros::Subscriber twist_left_sub;
    ros::Subscriber twist_right_sub;
    ros::Publisher arm_pos;
    
    // ros::Subscriber sub;

};

#endif // MAINWINDOW_H
