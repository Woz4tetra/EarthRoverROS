#!/usr/bin/python
from __future__ import division
import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Int64

class EarthRoverChassis:
    def __init__(self, ):
        rospy.init_node(
            "earth_rover_chassis",
            # log_level=rospy.DEBUG
        )
        # rospy.on_shutdown(self.shutdown)
        self.wheel_radius = rospy.get_param("~wheel_radius_meters", 1.0)
        self.wheel_distance = rospy.get_param("~wheel_distance_meters", 1.0)
        self.ticks_per_rotation = rospy.get_param("~ticks_per_rotation", 1.0)

        self.twist_sub = rospy.Subscriber("/cmd_vel", Twist, self.twist_callback, queue_size=5)
        self.left_encoder_sub = rospy.Subscriber("/left_encoder", Int64, self.left_encoder_callback, queue_size=50)
        self.right_encoder_sub = rospy.Subscriber("/right_encoder", Int64, self.right_encoder_callback, queue_size=50)

        self.left_setpoint_pub = rospy.Publisher("left/setpoint", Float64, queue_size=1)
        self.right_setpoint_pub = rospy.Publisher("right/setpoint", Float64, queue_size=1)

        self.left_state_pub = rospy.Publisher("left/state", Float64, queue_size=1)
        self.right_state_pub = rospy.Publisher("right/state", Float64, queue_size=1)

        self.ticks_to_meters = 2 * math.pi * self.wheel_radius / self.ticks_per_rotation

        self.left_speed_setpoint_mps = 0.0  # left motor goal speed in meters per second
        self.right_speed_setpoint_mps = 0.0  # right motor goal speed in meters per second

        self.left_enc_tick = 0
        self.right_enc_tick = 0

        self.prev_left_enc_tick = 0
        self.prev_right_enc_tick = 0
        self.prev_left_enc_time = rospy.Time.now()
        self.prev_right_enc_time = rospy.Time.now()

    # def shutdown(self):
    #     pass

    def twist_callback(self, twist_msg):
        linear_speed = twist_msg.linear.x
        angular_speed = twist_msg.angular.z

        self.left_speed_setpoint_mps = linear_speed - angular_speed
        self.right_speed_setpoint_mps = linear_speed + angular_speed

        self.left_setpoint_pub.publish(self.left_speed_setpoint_mps)
        self.right_setpoint_pub.publish(self.right_speed_setpoint_mps)

    def compute_speed(self, current_tick, prev_tick, current_time, prev_time):
        delta_tick = current_tick - prev_tick
        dt = (current_time - prev_time).to_sec()
        return delta_tick * self.ticks_to_meters / dt

    def compute_avg(self, rolling_avg_list):
        return sum(rolling_avg_list) / len(rolling_avg_list)

    def left_encoder_callback(self, enc_msg):
        self.left_enc_tick = enc_msg.data

    def right_encoder_callback(self, enc_msg):
        self.right_enc_tick = enc_msg.data

    def run(self):
        clock_rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.prev_left_enc_time).to_sec()
            d_tick = self.left_enc_tick - self.prev_left_enc_tick
            left_speed = self.compute_speed(self.left_enc_tick, self.prev_left_enc_tick, current_time, self.prev_left_enc_time)
            right_speed = self.compute_speed(self.left_enc_tick, self.prev_left_enc_tick, current_time, self.prev_left_enc_time)

            self.prev_left_enc_tick = self.left_enc_tick
            self.prev_right_enc_tick = self.right_enc_tick

            self.left_enc_tick = 0
            self.right_enc_tick = 0

            self.prev_left_enc_time = current_time
            self.prev_right_enc_time = current_time

            self.left_state_pub.publish(left_speed)
            self.right_state_pub.publish(right_speed)

            clock_rate.sleep()


if __name__ == "__main__":
    try:
        node = EarthRoverChassis()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting earth_rover_chassis node")
