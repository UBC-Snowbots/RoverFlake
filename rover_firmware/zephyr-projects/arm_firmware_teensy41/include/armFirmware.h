/*
Created By: Rowan Z
Created On: August 17, 2023
Updated On: 
Description: Header file for firmware for driving a 6 axis arm via ROS on a teensy 4.1 MCU
*/

//TODO
/*
crashes with more than 3 axis
weird shit in the buffer, but will just sanitize



*/
#pragma once
#include <stdlib.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>
//usb (needed for uart)
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include <zephyr/drivers/gpio.h>

#include <Axis.h>

#include <string>

// #define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
//#define UART_DEVICE DT_NODELABEL(lpuart4) 
// #define UART_DEV        DT_NODELABEL(lpuart4)
#define MSG_SIZE 32

// general parameters
#define NUM_AXES 6
#define ON 0
#define OFF 1
#define SW_ON 0
#define SW_OFF 1
#define FWD 1
#define REV 0

#define RING_BUF_SIZE 1024
#define RX_BUF_SIZE 64
#define TX_BUF_SIZE 128


// Motor pins     
inline int stepPins[6] =   {6, 8, 2, 10, 12, 25}; 
inline int dirPins[6] =    {5, 7, 1, 9, 26, 24}; 

// Encoder pins
inline int encPinA[6] = {17, 38, 40, 36, 13, 15};
inline int encPinB[6] = {16, 37, 39, 35, 41, 14};

// limit switch pins
inline int limPins[6] = {18, 19, 20, 21, 23, 22};

// pulses per revolution for motors
inline long ppr[6] = {400, 400, 400, 400, 400, 400};

// Gear Reductions
inline float red[6] = {50.0, 160.0, 92.3077, 43.936, 57.0, 5.18};

// End effector variables
const int closePos = 0; 
const int openPos = 90000; //TODO: needs to be set with proper step value for gear redcution and ppr
const int EEstepPin = 4;
const int EEdirPin = 3;
const int speedEE = 1000;
const int accEE = 1000;
const int MOTOR_DIR_EE = -1;
const int forcePin = 12;


// Encoder Variables
inline int curEncSteps[NUM_AXES], cmdEncSteps[NUM_AXES];
const int pprEnc = 512;
const float ENC_MULT[] = {5.12, 5.12, 5.12, 5.12, 5.12, 5.12};
inline float ENC_STEPS_PER_DEG[NUM_AXES];



//this creates the device for gpio pins
inline const struct device *gpio_dev;


inline const struct device *dev;
//inline char tx_msg[TX_BUF_SIZE];

void sendMsg(const char tx_msg[TX_BUF_SIZE]);

//timer callback to handle step signals
void stepper_timer_callback(struct k_timer *timer_id);

void parseCmd(uint8_t cmd[RX_BUF_SIZE]);
void initilizeAxis(struct InstAxis *instance);
void stepAxis(int axis, int steps);




