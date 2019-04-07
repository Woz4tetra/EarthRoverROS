#!/usr/bin/python
from __future__ import division
import math

import tf
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from actionlib_msgs.msg import GoalID

from earth_rover_microcontroller_bridge.srv import *


class EarthRoverTeleop:
    def __init__(self):
        rospy.init_node(
            "earth_rover_teleop",
            # log_level=rospy.DEBUG
        )

        # class variables
        self.twist_command = Twist()
        self.e_stop_lock_msg = Bool()
        self.e_stop_msg = Twist()
        self.is_e_stopped = False
        self.prev_joy_msg = None
        self.cancel_goal_msg = GoalID()
        self.current_pattern_index = 0
        self.light_patterns = ["w", "rs", "rf"]
        self.lights_are_on = True

        # button mapping:
        # 0: A,    1: B,     2: X,      3: Y
        # 4: L1,   5: R1,    6: Select, 7: Start
        # 8: Home, 9: L joy, 10: R joy
        self.buttons = {
            "A": 0,
            "B": 1,
            "X": 2,
            "Y": 3,
            "L1": 4,
            "R1": 5,
            "Select": 6,
            "Start": 7,
            "Home": 8,
            "L joy": 9,
            "R joy": 10
        }

        # parameters from launch file
        self.linear_scale = rospy.get_param("~linear_scale", 1.0)
        self.linear_axis = rospy.get_param("~linear_axis", 1)
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)
        self.angular_axis = rospy.get_param("~angular_axis", 2)
        self.publish_to_motors = rospy.get_param("~publish_directly_to_motors", False)
        self.twist_publish_topic_name = rospy.get_param("~twist_publish_topic_name", "cmd_vel")
        self.left_led_control_service_name = rospy.get_param("~left_led_control_service_name", "left/left_encoder/led_control_left")
        self.right_led_control_service_name = rospy.get_param("~right_led_control_service_name", "right/right_encoder/led_control_right")
        self.enable_move_base_hookups = rospy.get_param("~enable_move_base_hookups", False)
        self.cancel_move_base_goal_topic_name = "/move_base/cancel"

        # publishing topics
        self.cmd_vel_pub = None
        self.left_motor_pub = None
        self.right_motor_pub = None

        if self.publish_to_motors:
            self.left_motor_pub = rospy.Publisher("left/command_speed", Float64, queue_size=5)
            self.right_motor_pub = rospy.Publisher("right/command_speed", Float64, queue_size=5)
        else:
            self.cmd_vel_pub = rospy.Publisher(self.twist_publish_topic_name, Twist, queue_size=5)

        self.e_stop_lock_pub = rospy.Publisher("e_stop_lock", Bool, queue_size=5)
        self.e_stop_pub = rospy.Publisher("e_stop", Twist, queue_size=5)

        if self.enable_move_base_hookups:
            self.cancel_goal_pub = rospy.Publisher(self.cancel_move_base_goal_topic_name, GoalID, queue_size=1)
        else:
            self.cancel_goal_pub = None

        # subscription topics
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joystick_msg_callback, queue_size=5)

        # LED services
        rospy.loginfo("Waiting for service '%s'" % self.left_led_control_service_name)
        rospy.wait_for_service(self.left_led_control_service_name)
        rospy.loginfo("'%s' service is ready!" % self.left_led_control_service_name)

        rospy.loginfo("Waiting for service '%s'" % self.right_led_control_service_name)
        rospy.wait_for_service(self.right_led_control_service_name)
        rospy.loginfo("'%s' service is ready!" % self.right_led_control_service_name)

        self.left_led_control_service = rospy.ServiceProxy(self.left_led_control_service_name, LedControl)
        self.right_led_control_service = rospy.ServiceProxy(self.right_led_control_service_name, LedControl)
        rospy.loginfo("Joystick E stop is mapped to the Y button")

    def did_button_change(self, msg, index):
        return msg.buttons[index] and self.prev_joy_msg.buttons[index] != msg.buttons[index]

    def joystick_msg_callback(self, msg):
        if self.prev_joy_msg is None:
            self.prev_joy_msg = msg
            return

        # button mapping:
        # 0: A,    1: B,     2: X,      3: Y
        # 4: L1,   5: R1,    6: Select, 7: Start
        # 8: Home, 9: L joy, 10: R joy
        if self.did_button_change(msg, self.buttons["A"]):  # toggle lights
            self.lights_are_on = not self.lights_are_on
            if self.lights_are_on:
                self.set_led_pattern()
            else:
                self.control_leds("o")

        elif self.did_button_change(msg, self.buttons["B"]):  # switch light pattern
            if self.lights_are_on:
                self.set_next_pattern()
                self.set_led_pattern()

        elif self.did_button_change(msg, self.buttons["R1"]):
            self.is_e_stopped = not self.is_e_stopped
            self.e_stop_lock_msg.data = self.is_e_stopped
            if self.is_e_stopped:
                rospy.logwarn("Joystick E stop was thrown!")
            else:
                rospy.loginfo("Joystick E stop was released")

        elif self.did_button_change(msg, self.buttons["L1"]):
            if self.enable_move_base_hookups:
                self.cancel_goal_pub.publish(self.cancel_goal_msg)
                print "Canceling goal from joystick button press"

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

        self.prev_joy_msg = msg

    def set_led_pattern(self):
        self.control_leds(self.light_patterns[self.current_pattern_index])

    def set_next_pattern(self):
        self.current_pattern_index += 1
        if self.current_pattern_index >= len(self.light_patterns):
            self.current_pattern_index = 0

    def control_leds(self, command):
        try:
            self.left_led_control_service(command)
            self.right_led_control_service(command)

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def run(self):
        clock_rate = rospy.Rate(30)
        prev_time = rospy.get_rostime()
        self.set_led_pattern()

        while not rospy.is_shutdown():
            if self.is_e_stopped:
                self.e_stop_pub.publish(self.e_stop_msg)

                now = rospy.get_rostime()
                if now - prev_time > rospy.Duration(1):
                    rospy.logwarn("Joystick E stop is thrown. Press R1 to release")
                    prev_time = now

            self.e_stop_lock_pub.publish(self.e_stop_lock_msg)
            clock_rate.sleep()


if __name__ == "__main__":
    try:
        node = EarthRoverTeleop()
        node.run()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting earth_rover_chassis node")
