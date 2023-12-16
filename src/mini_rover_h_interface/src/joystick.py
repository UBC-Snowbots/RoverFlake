#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String

LOWRANGE = -250
UPRANGE = 250

def scale(val, src, dst):
    return ((val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

def joy_callback(data, pub):
    # Extract axis inputs
    axes = data.axes
    motor_left = 0.0
    motor_right = 0.0

    # Perform any processing on the axis inputs if needed
    # For simplicity, let's just publish the first axis value to a new topic
    # axes[0] = left right
    # axes[1] = up down
    if axes:
        if axes[3] > 0.50 or axes[3] < -0.5:
            motor_left = scale(axes[3], (1.0, -1.0), (LOWRANGE, UPRANGE))
            motor_right =  scale(axes[3], (-1.0, 1.0), (LOWRANGE, UPRANGE))

        elif axes[0] == 0:
            motor_left = scale(axes[1], (-1.0, 1.0), (LOWRANGE, UPRANGE))
            motor_right =  scale(axes[1], (-1.0, 1.0), (LOWRANGE, UPRANGE))

        elif axes[1] == 0:
            if axes[0] == -1.0: 
                motor_right = UPRANGE
                motor_left = 0
            if axes[0] == 1.0:
                motor_left = UPRANGE
                motor_right = 0
        else:
            if axes[0] < 0:
                motor_right = scale(axes[1], (-1.0, 1.0), (LOWRANGE, UPRANGE))
                motor_left = 0.5 * scale(axes[1], (-1.0, 1.0), (LOWRANGE, UPRANGE))
            if axes[0] >= 0:
                motor_left = scale(axes[1], (-1.0, 1.0), (LOWRANGE, UPRANGE))
                motor_right = 0.5 * scale(axes[1], (-1.0, 1.0), (LOWRANGE, UPRANGE))

        

        motor_str = f"$V({round(motor_left, 3)},{round(motor_left, 3)},{round(motor_left, 3)},{round(motor_right, 3)},{round(motor_right, 3)},{round(motor_right, 3)})"

        pub.publish(motor_str)

def joy_controller():
    rospy.init_node('joy_controller', anonymous=True)
    

    # Create a publisher for the filtered_joy topic
    pub = rospy.Publisher('mini_rover_data', String, queue_size=10)

    # Set the callback function for the joy topic
    rospy.Subscriber('joy', Joy, joy_callback, pub)

    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        joy_controller()
    except rospy.ROSInterruptException:
        pass
