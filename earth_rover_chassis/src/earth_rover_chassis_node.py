#!/usr/bin/python
from __future__ import division
import math

import tf
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64, Int64

from motor_controller import MotorController, MotorInfo
from earth_rover_chassis.srv import TuneMotorPID, TuneMotorPIDResponse

class EarthRoverChassis:
    def __init__(self):
        rospy.init_node(
            "earth_rover_chassis",
            # log_level=rospy.DEBUG
        )

        # rospy.on_shutdown(self.shutdown)
        self.wheel_radius = rospy.get_param("~wheel_radius_meters", 1.0)
        self.wheel_distance = rospy.get_param("~wheel_distance_meters", 1.0)
        self.ticks_per_rotation = rospy.get_param("~ticks_per_rotation", 1.0)
        self.left_min_speed_meters_per_s = rospy.get_param("~left_min_speed_meters_per_s", -1.0)
        self.left_max_speed_meters_per_s = rospy.get_param("~left_max_speed_meters_per_s", 1.0)
        self.right_min_speed_meters_per_s = rospy.get_param("~right_min_speed_meters_per_s", -1.0)
        self.right_max_speed_meters_per_s = rospy.get_param("~right_max_speed_meters_per_s", 1.0)
        self.min_command = rospy.get_param("~min_command", -1.0)
        self.max_command = rospy.get_param("~max_command", 1.0)
        self.kp = rospy.get_param("~kp", 1.0)
        self.ki = rospy.get_param("~ki", 0.0)
        self.kd = rospy.get_param("~kd", 0.0)
        self.speed_smooth_k = rospy.get_param("~speed_smooth_k", 1.0)
        self.output_deadzone = rospy.get_param("~output_deadzone", 0.01)
        self.child_frame = rospy.get_param("~odom_child_frame", "base_link")
        self.parent_frame = rospy.get_param("~odom_parent_frame", "odom")
        self.enable_pid = rospy.get_param("~enable_pid", True)

        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=5)
        self.left_encoder_sub = rospy.Subscriber("left/left_encoder/ticks", Int64, self.left_encoder_callback, queue_size=50)
        self.right_encoder_sub = rospy.Subscriber("right/right_encoder/ticks", Int64, self.right_encoder_callback, queue_size=50)

        self.left_dist_pub = rospy.Publisher("left/left_encoder/distance", Float64, queue_size=5)
        self.right_dist_pub = rospy.Publisher("right/right_encoder/distance", Float64, queue_size=5)
        self.left_speed_pub = rospy.Publisher("left/left_encoder/speed", Float64, queue_size=5)
        self.right_speed_pub = rospy.Publisher("right/right_encoder/speed", Float64, queue_size=5)

        self.left_command_pub = rospy.Publisher("left/command_speed", Float64, queue_size=5)
        self.right_command_pub = rospy.Publisher("right/command_speed", Float64, queue_size=5)

        self.tuning_service = rospy.Service("tune_motor_pid", TuneMotorPID, self.tune_motor_pid)

        self.tf_broadcaster = tf.TransformBroadcaster()

        left_motor_info = MotorInfo(
            "left motor",
            self.kp, self.ki, self.kd, self.speed_smooth_k,
            self.wheel_radius, self.ticks_per_rotation,
            self.left_min_speed_meters_per_s, self.left_max_speed_meters_per_s,
            self.min_command, self.max_command, self.output_deadzone
        )

        right_motor_info = MotorInfo(
            "right motor",
            self.kp, self.ki, self.kd, self.speed_smooth_k,
            self.wheel_radius, self.ticks_per_rotation,
            self.right_min_speed_meters_per_s, self.right_max_speed_meters_per_s,
            self.min_command, self.max_command, self.output_deadzone
        )

        self.left_motor = MotorController(left_motor_info)
        self.right_motor = MotorController(right_motor_info)

        self.prev_enc_time = rospy.Time.now()

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_t = 0.0

        self.prev_left_output = 0.0
        self.prev_right_output = 0.0

    # def shutdown(self):
    #     pass

    def tune_motor_pid(self, request):
        kp = request.kp
        ki = request.ki
        kd = request.kd
        rospy.loginfo("Updated PID constants to kp=%s, ki=%s, kd=%s" % (kp, ki, kd))

        self.left_motor.tune_pid(kp, ki, kd)
        self.right_motor.tune_pid(kp, ki, kd)

        return TuneMotorPIDResponse()

    def twist_callback(self, twist_msg):
        linear_speed_mps = twist_msg.linear.x  # m/s
        angular_speed_radps = twist_msg.angular.z  # rad/s

        # arc = angle * radius
        # rotation speed at the wheels
        rotational_speed_mps = angular_speed_radps / (2 * math.pi) * self.wheel_distance / 2

        self.left_motor.set_target(linear_speed_mps - rotational_speed_mps)
        self.right_motor.set_target(linear_speed_mps + rotational_speed_mps)

    def left_encoder_callback(self, enc_msg):
        self.left_motor.enc_tick = -enc_msg.data

    def right_encoder_callback(self, enc_msg):
        self.right_motor.enc_tick = enc_msg.data

    def run(self):
        clock_rate = rospy.Rate(30)

        self.prev_enc_time = rospy.Time.now()
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.prev_enc_time).to_sec()
            self.prev_enc_time = current_time

            left_output = self.left_motor.update(dt)
            right_output = self.right_motor.update(dt)

            if self.prev_left_output != left_output or self.prev_right_output != right_output:
                self.command_motors(left_output, right_output)
                self.prev_left_output = left_output
                self.prev_right_output = right_output

            self.compute_odometry_pose(self.left_motor.delta_dist, self.right_motor.delta_dist)

            self.publish_chassis_data()

            clock_rate.sleep()

    def command_motors(self, left_command, right_command):
        if self.enable_pid:
            self.left_command_pub.publish(left_command)
            self.right_command_pub.publish(right_command)

    def publish_chassis_data(self):
        self.left_dist_pub.publish(self.left_motor.get_dist())
        self.right_dist_pub.publish(self.right_motor.get_dist())
        self.left_speed_pub.publish(self.left_motor.get_speed())
        self.right_speed_pub.publish(self.right_motor.get_speed())

        self.tf_broadcaster.sendTransform(
            (self.odom_x, self.odom_y, 0.0),
            tf.transformations.quaternion_from_euler(0.0, 0.0, self.odom_t),
            rospy.Time.now(),
            self.child_frame,
            self.parent_frame
        )

    def compute_odometry_pose(self, delta_left, delta_right):
        delta_dist = (delta_left + delta_right) / 2

        # angle = arc / radius
        delta_angle = (delta_right - delta_left) / (self.wheel_distance / 2)

        dx = delta_dist * math.cos(self.odom_t)
        dy = delta_dist * math.sin(self.odom_t)

        self.odom_x += dx
        self.odom_y += dy
        self.odom_t += delta_angle

if __name__ == "__main__":
    try:
        node = EarthRoverChassis()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting earth_rover_chassis node")
