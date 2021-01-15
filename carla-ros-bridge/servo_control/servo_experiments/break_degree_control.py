#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def servo_fine():
    pub = rospy.Publisher('breakServo', Float32, queue_size=10)
    rospy.init_node('servo_controller', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        angle = input("Angle (0-270): ")
	rospy.loginfo(angle)
        pub.publish(angle)
        rate.sleep()

if __name__ == '__main__':
    try:
        servo_fine()
    except rospy.ROSInterruptException:
        pass