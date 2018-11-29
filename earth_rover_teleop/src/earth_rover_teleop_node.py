#!/usr/bin/python
from __future__ import division
import math

import tf
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class EarthRoverTeleop:
    def __init__(self):
        rospy.init_node(
            "earth_rover_teleop",
            # log_level=rospy.DEBUG
        )

        self.linear_scale = rospy.get_param("~linear_scale", 1.0)
        self.linear_axis = rospy.get_param("~linear_axis", 1)
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)
        self.angular_axis = rospy.get_param("~angular_axis", 2)
        self.publish_to_motors = rospy.get_param("~publish_directly_to_motors", False)

        self.cmd_vel_pub = None
        self.left_motor_pub = None
        self.right_motor_pub = None
        if self.publish_to_motors:
            self.left_motor_pub = rospy.Publisher("left/command_speed", Float64, queue_size=5)
            self.right_motor_pub = rospy.Publisher("right/command_speed", Float64, queue_size=5)
        else:
            self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joystick_msg_callback, queue_size=5)

        self.twist_command = Twist()

    def joystick_msg_callback(self, msg):
        linear_val = self.linear_scale * msg.axes[int(self.linear_axis)]
        angular_val = self.angular_scale * msg.axes[int(self.angular_axis)]
        if self.publish_to_motors:
            self.left_motor_pub.publish(linear_val - angular_val)
            self.right_motor_pub.publish(linear_val + angular_val)
            print "left joy: %s" % (linear_val - angular_val)
            print "right joy: %s" % (linear_val + angular_val)
        else:
            self.twist_command.linear.x = linear_val
            self.twist_command.angular.z = angular_val
            self.cmd_vel_pub.publish(self.twist_command)

if __name__ == "__main__":
    try:
        EarthRoverTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting earth_rover_chassis node")
