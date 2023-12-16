#!/usr/bin/env python3

#DEBUGGING NODE (publisher that sends valid string to subscriber)

import rospy
from std_msgs.msg import String

def mini_rover_debug():
    pub = rospy.Publisher('mini_rover_data', String, queue_size=10)
    rospy.init_node('mini_rover_debug', anonymous=True)
    rate = rospy.Rate(4) #10 Hz

    while not rospy.is_shutdown():
        debug_str = "$V(110,2,3,4,5,6)" #valid string that powers motor 1 
        rospy.loginfo(debug_str)
        pub.publish(debug_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        mini_rover_debug()
    except rospy.ROSInterruptException:
        pass
