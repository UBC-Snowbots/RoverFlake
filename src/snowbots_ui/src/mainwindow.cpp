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

#define INIT_STATUS -10
#define OFFLINE 0
#define STANDBY 5
#define ONLINE 1
#define ERROR -1

std_msgs::Float64 elec_box_tempurature_msg;
_Float64 elec_box_tempurature = 2.0;
std::string ROVER_GUI_MODE = "Generic, WARN: UNSET";
int16_t arm_status = INIT_STATUS;//INIT_STATUS; 
int16_t network_ags_status = INIT_STATUS;
int16_t gnss_status = INIT_STATUS;
int16_t rover_status = INIT_STATUS;
int16_t utility_mcu_status = INIT_STATUS;

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent),
    ui(new Ui::MainWindow),
    n{} { // Directly initializing NodeHandle with {}
    arm_pos = n.advertise<std_msgs::String>("chatter", 10);
    ui->setupUi(this);
    this->setWindowTitle("Snowbots Interface");
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update_UI()));
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
    elec_box_tempurature_sub = n.subscribe<std_msgs::Float64>("/rover_utility/elec_box_temperature", 
         5, boost::bind(&MainWindow::elec_box_temp_callback, this, _1));
  
    arm_status_sub = n.subscribe<std_msgs::Int16>("/status/arm_mcu", 
         5, boost::bind(&MainWindow::arm_status_callback, this, _1));
    
    rover_status_sub = n.subscribe<std_msgs::Int16>("/status/rover_computer", 
         5, boost::bind(&MainWindow::rover_status_callback, this, _1));
    
    gnss_status_sub = n.subscribe<std_msgs::Int16>("/status/gnss", 
         5, boost::bind(&MainWindow::gnss_status_callback, this, _1));
    
    network_ags_status_sub  = n.subscribe<std_msgs::Int16>("/status/network_ags", 
         5, boost::bind(&MainWindow::network_ags_status_callback, this, _1));
    
    ROS_INFO("UBC ROVER GUI START APEARS SUCCESSFULL");
    if(system("which toilet >/dev/null 2>&1") != 0) {
        std::cout << "toilet is not installed, falling back to regular text. run - sudo apt-get install toilet - if you want a more fun starting message\n";
        std::cout << "Rover GUI has started! Check for the new window\n";
    } else {
        system("toilet -f future -F gay -F crop 'UBC ROVER GUI STARTED!'");
    }

}

MainWindow::~MainWindow() {
    delete ui;
    //ros::Rate(10).sleep(); //probably want to make this ros::Rate, nvm, i have no idea how qt-ros works lol
    qDebug() << "Destructor OK";
    
    
}


void MainWindow::update_UI() {
 ui->chassis_velocity_dial->setValue(0.0);
    ui->arm_battery_dial->setValue(0.0);
  if(ui_debug){
    ROS_INFO("UI is still listening, %f", elec_box_tempurature);
  }
    ui->mode_lcd->setText(ROVER_GUI_MODE.c_str()); // replace hellow world with message for mode
    ui->nuc_temp_lcd->display(elec_box_tempurature_msg.data); // message req for updating nuc temp
    //ui->rover_temp_lcd->display(elec_box_tempurature_msg.data); // message req for updating rover temp
   
    ui->end_effector_force_feedback_lcd->display(twist_message_controller.linear.x); // message req for updating end effector force feeback as % of grip
    ui->end_effector_pos_feedback_lcd->display(twist_message_controller.linear.x); // message req for updating end effector position feedback
    //ros_f->twist_subscriber(); // message req for updating elec box temp
   
   //elec box temp
    QPalette thermPalette = ui->elec_box_temp_therm->palette();
    thermPalette.setColor(QPalette::Base, Qt::white);
    if(elec_box_tempurature < 25){
    thermPalette.setColor(QPalette::ButtonText, Qt::blue);
    } else if(elec_box_tempurature < 30){
    thermPalette.setColor(QPalette::ButtonText, Qt::green);
    } else if(elec_box_tempurature < 43){
    thermPalette.setColor(QPalette::ButtonText, Qt::yellow);
    }
    else {
    thermPalette.setColor(QPalette::ButtonText, Qt::red);
    }
    thermPalette.setColor(QPalette::Highlight, Qt::black); 
    ui->elec_box_temp_therm->setPalette(thermPalette);

    ui->elec_box_temp_therm->setValue(elec_box_tempurature);
    ui->elec_box_temp_lcd->display(elec_box_tempurature); // message req for updating elec box temp

    QPalette statusPallette = ui->rover_comp_status_label->palette();
   
    switch (arm_status)
    {
    case INIT_STATUS:
      statusPallette.setColor(QPalette::WindowText, Qt::darkGray);
      ui->arm_status_label->setPalette(statusPallette);
      ui->arm_status_label->setText("starting");
      break;
    case OFFLINE:
      statusPallette.setColor(QPalette::WindowText, Qt::darkRed);
      ui->arm_status_label->setPalette(statusPallette);
      ui->arm_status_label->setText("offline");
      break;
    case STANDBY:
      statusPallette.setColor(QPalette::WindowText, Qt::darkYellow);
      ui->arm_status_label->setPalette(statusPallette);
      ui->arm_status_label->setText("STANDBY");
      break;
    case ONLINE:
      statusPallette.setColor(QPalette::WindowText, Qt::darkGreen);
      ui->arm_status_label->setPalette(statusPallette);
      ui->arm_status_label->setText("online");
      break;
    default:
      statusPallette.setColor(QPalette::WindowText, Qt::red);
      ui->arm_status_label->setPalette(statusPallette);
      ui->arm_status_label->setText("ERROR");
      break;
    }
  
    switch (rover_status)
    {
    case INIT_STATUS:
      statusPallette.setColor(QPalette::WindowText, Qt::darkGray);
      ui->rover_comp_status_label->setPalette(statusPallette);
      ui->rover_comp_status_label->setText("starting");
      break;
    case OFFLINE:
      statusPallette.setColor(QPalette::WindowText, Qt::darkRed);
      ui->rover_comp_status_label->setPalette(statusPallette);
      ui->rover_comp_status_label->setText("offline");
      break;
    case STANDBY:
      statusPallette.setColor(QPalette::WindowText, Qt::darkYellow);
      ui->rover_comp_status_label->setPalette(statusPallette);
      ui->rover_comp_status_label->setText("STANDBY");
      break;
    case ONLINE:
      statusPallette.setColor(QPalette::WindowText, Qt::darkGreen);
      ui->rover_comp_status_label->setPalette(statusPallette);
      ui->rover_comp_status_label->setText("online");
      break;
    default:
      statusPallette.setColor(QPalette::WindowText, Qt::red);
      ui->rover_comp_status_label->setPalette(statusPallette);
      ui->rover_comp_status_label->setText("ERROR");
      break;
    }

    switch (gnss_status)
    {
    case INIT_STATUS:
      statusPallette.setColor(QPalette::WindowText, Qt::darkGray);
      ui->gnss_status_label->setPalette(statusPallette);
      ui->gnss_status_label->setText("starting");
      break;
    case OFFLINE:
      statusPallette.setColor(QPalette::WindowText, Qt::darkRed);
      ui->gnss_status_label->setPalette(statusPallette);
      ui->gnss_status_label->setText("offline");
      break;
    case STANDBY:
      statusPallette.setColor(QPalette::WindowText, Qt::darkYellow);
      ui->gnss_status_label->setPalette(statusPallette);
      ui->gnss_status_label->setText("STANDBY");
      break;
    case ONLINE:
      statusPallette.setColor(QPalette::WindowText, Qt::darkGreen);
      ui->gnss_status_label->setPalette(statusPallette);
      ui->gnss_status_label->setText("online");
      break;
    default:
      statusPallette.setColor(QPalette::WindowText, Qt::red);
      ui->gnss_status_label->setPalette(statusPallette);
      ui->gnss_status_label->setText("ERROR");
      break;
    }

    switch (network_ags_status)
    {
    case INIT_STATUS:
      statusPallette.setColor(QPalette::WindowText, Qt::darkGray);
      ui->net_ags_status_label->setPalette(statusPallette);
      ui->net_ags_status_label->setText("starting");
      break;
    case OFFLINE:
      statusPallette.setColor(QPalette::WindowText, Qt::darkRed);
      ui->net_ags_status_label->setPalette(statusPallette);
      ui->net_ags_status_label->setText("offline");
      break;
    case STANDBY:
      statusPallette.setColor(QPalette::WindowText, Qt::darkYellow);
      ui->net_ags_status_label->setPalette(statusPallette);
      ui->net_ags_status_label->setText("STANDBY FOR GNSS");
      break;
    case ONLINE:
      statusPallette.setColor(QPalette::WindowText, Qt::darkGreen);
      ui->net_ags_status_label->setPalette(statusPallette);
      ui->net_ags_status_label->setText("online");
      break;
    default:
      statusPallette.setColor(QPalette::WindowText, Qt::red);
      ui->net_ags_status_label->setPalette(statusPallette);
      ui->net_ags_status_label->setText("ERROR");
      break;
    }

    switch (utility_mcu_status)
    {
    case INIT_STATUS:
      statusPallette.setColor(QPalette::WindowText, Qt::darkGray);
      ui->utility_mcu_status_label->setPalette(statusPallette);
      ui->utility_mcu_status_label->setText("starting");
      break;
    case OFFLINE:
      statusPallette.setColor(QPalette::WindowText, Qt::darkRed);
      ui->utility_mcu_status_label->setPalette(statusPallette);
      ui->utility_mcu_status_label->setText("offline");
      break;
    case STANDBY:
      statusPallette.setColor(QPalette::WindowText, Qt::darkYellow);
      ui->utility_mcu_status_label->setPalette(statusPallette);
      ui->utility_mcu_status_label->setText("STANDBY");
      break;
    case ONLINE:
      statusPallette.setColor(QPalette::WindowText, Qt::darkGreen);
      ui->utility_mcu_status_label->setPalette(statusPallette);
      ui->utility_mcu_status_label->setText("online");
      break;
    default:
      statusPallette.setColor(QPalette::WindowText, Qt::red);
      ui->utility_mcu_status_label->setPalette(statusPallette);
      ui->utility_mcu_status_label->setText("ERROR");
      break;
    }
    
   

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