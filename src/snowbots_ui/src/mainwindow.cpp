/*
 * Created By: Adam Nguyen
 * Created On: August 21st, 2021
 * Snowbots UI
 */

#include "../include/mainwindow.h"
#include "ui_mainwindow.h"

// QT
#include "QDebug"
#include "QDir"
#include "QLabel"
#include "QMessageBox"
#include "QPixmap"
#include "QProcess"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
std_msgs::Float64 elec_box_tempurature_msg;
_Float64 elec_box_tempurature = 2.0;

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent),
    ui(new Ui::MainWindow),
    n{} { // Directly initializing NodeHandle with {}
    arm_pos = n.advertise<std_msgs::String>("chatter", 1000);
    ui->setupUi(this);
    this->setWindowTitle("Snowbots Interface");
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(twist_values()));
    timer->start(100);

    // ROS
    qDebug() << "Constructor OK";

    // UI
    connect(ui->Pos1_button, &QPushButton::released, this, &MainWindow::handleButton1);
    // connect(ui->Pos2_button, &QPushButton::released, this, &MainWindow::handleButton2);
    // connect(ui->Pos3_button, &QPushButton::released, this, &MainWindow::handleButton3);
    // connect(ui->Pos4_button, &QPushButton::released, this, &MainWindow::handleButton4);
    // connect(ui->Pos5_button, &QPushButton::released, this, &MainWindow::handleButton5);
    // connect(ui->Pos6_button, &QPushButton::released, this, &MainWindow::handleButton6);
    // Snowbots Logo
    QPixmap pixmap("./src/snowbots_ui/resources/snowbot2.png");
    ui->label_5->setPixmap(pixmap);
    ui->label_5->show();
    ui->label_5->setScaledContents(true);
    qDebug() << "Current dir:" << QDir::currentPath();

    // Initialize ROS subscriber after n has been initialized
    twist_left_sub = n.subscribe(
        "/integration_node/lwheels_pub_topic", 1000, MainWindow::twist_left_callback);
    twist_right_sub = n.subscribe(
        "/integration_node/rwheels_pub_topic", 1000, MainWindow::twist_right_callback);
    elec_box_tempurature_sub = n.subscribe<std_msgs::Float64>("/rover_utility/elec_box_tempurature", 
         10, boost::bind(&MainWindow::elec_box_temp_callback, this, _1));
}

MainWindow::~MainWindow() {
    delete ui;
    ros::Rate(10).sleep(); //probably want to make this ros::Rate
    qDebug() << "Destructor OK";
}


void MainWindow::twist_values() {
    ROS_INFO("UI is still listening, %f", elec_box_tempurature);

    ui->mode_lcd->setText("hello"); // replace hellow world with message for mode
    ui->nuc_temp_lcd->display(elec_box_tempurature_msg.data); // message req for updating nuc temp
    ui->rover_temp_lcd->display(elec_box_tempurature_msg.data); // message req for updating rover temp
    ui->elec_box_temp_lcd->display(elec_box_tempurature); // message req for updating elec box temp
    ui->end_effector_force_feedback_lcd->display(twist_message_controller.linear.x); // message req for updating end effector force feeback as % of grip
    ui->end_effector_pos_feedback_lcd->display(twist_message_controller.linear.x); // message req for updating end effector position feedback
    //ros_f->twist_subscriber(); // message req for updating elec box temp
   ros::spinOnce();
}
// TODOS: 
// Current Mode - status - done
// end effector position feedback (hopefully) - ui ok
// end effector force feedback as % of grip - ui ok
// nuc temperature - done
// rover ambient temp - done
// elec box temp - done
// stored arm poses (button) - added in the ui, but still need to figure out for ros integration

//TODO: ros inegration: get texts/numbres and send messages 

void MainWindow::handleButton2()
{
  // change the text
  ui->Pos2_button->setText("Clicked!");
  // resize button
}
void MainWindow::handleButton3()
{
  // change the text
  ui->Pos3_button->setText("Clicked!");
  // resize button
}
void MainWindow::handleButton4()
{
  // change the text
  ui->Pos4_button->setText("Clicked!");
  // resize button
}
void MainWindow::handleButton5()
{
  // change the text
  ui->Pos5_button->setText("Clicked!");
  // resize button
}
void MainWindow::handleButton6()
{
  // change the text
  ui->Pos6_button->setText("Clicked!");
  // resize button
}