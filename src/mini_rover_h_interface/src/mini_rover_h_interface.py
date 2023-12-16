#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import serial

#Function to subscribe to topic and send data over serial
def serial_node():
    rospy.init_node('mini_rover_h_interface', anonymous=True)

    serial_port = '/dev/ttyUSB0'
    baud_rate = 115200 

    try:
        ser = serial.Serial(serial_port, baud_rate)

        #define a callback function to handle incoming string data
        def string_callback(data):
            #this just logs the data
            rospy.loginfo("Received data: %s", data.data)
            #encode using unicode and send over serial
            ser.write(data.data.encode('utf-8'))

        #initialize subscriber to 'mini_rover_data' topic
        rospy.Subscriber('mini_rover_data', String, string_callback)
        rospy.spin()

    except serial.SerialException as e:
        rospy.logerr("Error opening the serial port: %s", str(e))
    finally:
        # Close the serial port when done
        ser.close()

if __name__ == '__main__':
    serial_node()
