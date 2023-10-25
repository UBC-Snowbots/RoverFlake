#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String

received_data = "" #String to store recieved data

#callback function to store recieved data to variable
def callback(data):
    global received_data
    received_data = data.data
    rospy.loginfo("Received data = %s", received_data)

#send data over serial to arduino
def send_serial_data(data_to_send):
    arduino = serial.Serial(port="/dev/ttyACM0") 
    arduino.write(bytes(data_to_send, 'utf-8'))
    arduino.close()

#initiliaze node and run functions
def mini_rover_interface():
    rospy.init_node('mini_rover_interface', anonymous=True)
    rospy.Subscriber('mini_rover_data', String, callback)
    rospy.spin()

    while not rospy.is_shutdown(): #while ROS is running, send data over serial
        send_serial_data(received_data)
        rospy.sleep(1)

if __name__ == '__main__':
    mini_rover_interface()
    