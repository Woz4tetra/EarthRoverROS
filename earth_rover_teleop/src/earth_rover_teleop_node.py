#!/usr/bin/python
from __future__ import division
import math

import tf
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from earth_rover_microcontroller_bridge.srv import *

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
        self.left_led_control_service_name = rospy.get_param("~left_led_control_service_name", "left/left_encoder/led_control_left")
        self.right_led_control_service_name = rospy.get_param("~right_led_control_service_name", "right/right_encoder/led_control_right")

        self.cmd_vel_pub = None
        self.left_motor_pub = None
        self.right_motor_pub = None
        if self.publish_to_motors:
            self.left_motor_pub = rospy.Publisher("left/command_speed", Float64, queue_size=5)
            self.right_motor_pub = rospy.Publisher("right/command_speed", Float64, queue_size=5)
        else:
            self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joystick_msg_callback, queue_size=5)

        rospy.loginfo("Waiting for service '%s'" % self.left_led_control_service_name)
        rospy.wait_for_service(self.left_led_control_service_name)
        rospy.loginfo("'%s' service is ready!" % self.left_led_control_service_name)

        rospy.loginfo("Waiting for service '%s'" % self.right_led_control_service_name)
        rospy.wait_for_service(self.right_led_control_service_name)
        rospy.loginfo("'%s' service is ready!" % self.right_led_control_service_name)

        self.left_led_control_service = rospy.ServiceProxy(self.left_led_control_service_name, LedControl)
        self.right_led_control_service = rospy.ServiceProxy(self.right_led_control_service_name, LedControl)

        self.twist_command = Twist()

    def joystick_msg_callback(self, msg):
        if msg.buttons[1]:  # B
            self.control_leds("w")

        elif msg.buttons[0]:  # A
            self.control_leds("o")

        elif msg.buttons[2]:  # X
            self.control_leds("rs")

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

    def control_leds(self, command):
        try:
            self.left_led_control_service(command)
            self.right_led_control_service(command)

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

if __name__ == "__main__":
    try:
        EarthRoverTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting earth_rover_chassis node")