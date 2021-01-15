#!/usr/bin/env python

import rospy
from carla_msgs.msg import CarlaEgoVehicleControl
from std_msgs.msg import Float32




def servo_fine():
    throttle_pub = rospy.Publisher('acceleratorServo', Float32, queue_size=10)
    break_pub = rospy.Publisher('breakServo', Float32, queue_size=10)
    steer_pub = rospy.Publisher('steeringServo', Float32, queue_size=10)
    rospy.init_node('servo_controller', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
	code_control = CarlaEgoVehicleControl()
        throttle_angle = code_control.throttle * 270
        steer_angle = code_control.steer * 270
        break_angle = code_control.brake * 270

	rospy.loginfo(throttle_angle)
        rospy.loginfo(break_angle)
        rospy.loginfo(steer_angle)

        throttle_pub.publish(throttle_angle)
        break_pub.publish(break_angle)
        steer_pub.publish(steer_angle)

        rate.sleep()

if __name__ == '__main__':
    try:
        servo_fine()
    except rospy.ROSInterruptException:
        pass
