#!/usr/bin/python
from __future__ import division
import math

import tf
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64, Int64, Float32MultiArray
from nav_msgs.msg import Odometry

from motor_controller import MotorController, MotorInfo
from ultrasonic_tracker import UltrasonicTracker, Direction
from earth_rover_chassis.srv import TuneMotorPID, TuneMotorPIDResponse

class EarthRoverChassis:
    def __init__(self):
        rospy.init_node(
            "earth_rover_chassis",
            # log_level=rospy.DEBUG
        )

        # rospy.on_shutdown(self.shutdown)

        # robot dimensions
        self.wheel_radius = rospy.get_param("~wheel_radius_meters", 1.0)
        self.wheel_distance = rospy.get_param("~wheel_distance_meters", 1.0)
        self.ticks_per_rotation = rospy.get_param("~ticks_per_rotation", 1.0)

        # speed parameters
        self.left_min_speed_meters_per_s = rospy.get_param("~left_min_speed_meters_per_s", -1.0)
        self.left_max_speed_meters_per_s = rospy.get_param("~left_max_speed_meters_per_s", 1.0)
        self.right_min_speed_meters_per_s = rospy.get_param("~right_min_speed_meters_per_s", -1.0)
        self.right_max_speed_meters_per_s = rospy.get_param("~right_max_speed_meters_per_s", 1.0)

        # PID parameters
        self.enable_pid = rospy.get_param("~enable_pid", True)
        self.min_command = rospy.get_param("~min_command", -1.0)
        self.max_command = rospy.get_param("~max_command", 1.0)
        self.kp = rospy.get_param("~kp", 1.0)
        self.ki = rospy.get_param("~ki", 0.0)
        self.kd = rospy.get_param("~kd", 0.0)
        self.speed_smooth_k = rospy.get_param("~speed_smooth_k", 1.0)
        self.output_deadzone = rospy.get_param("~output_deadzone", 0.01)

        # TF parameters
        self.child_frame = rospy.get_param("~odom_child_frame", "base_link")
        self.parent_frame = rospy.get_param("~odom_parent_frame", "odom")
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Ultrasonic parameters
        self.stopping_distances_cm = rospy.get_param("~stopping_distances_cm", None)
        self.easing_offset_cm = rospy.get_param("~easing_offset_cm", 5.0)
        self.front_sensors = rospy.get_param("~front_sensors", None)
        self.right_sensors = rospy.get_param("~right_sensors", None)
        self.left_sensors = rospy.get_param("~left_sensors", None)
        self.back_sensors = rospy.get_param("~back_sensors", None)

        assert self.stopping_distances_cm is not None
        assert self.front_sensors is not None
        assert self.right_sensors is not None
        assert self.left_sensors is not None
        assert self.back_sensors is not None

        # Subscribers
        self.twist_sub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback, queue_size=5)
        self.left_encoder_sub = rospy.Subscriber("left/left_encoder/ticks", Int64, self.left_encoder_callback, queue_size=50)
        self.right_encoder_sub = rospy.Subscriber("right/right_encoder/ticks", Int64, self.right_encoder_callback, queue_size=50)
        self.ultrasonic_sub = rospy.Subscriber(
            "earth_rover_teensy_bridge/ultrasonic",
            Float32MultiArray, self.ultrasonic_callback, queue_size=15
        )

        # Publishers
        self.left_dist_pub = rospy.Publisher("left/left_encoder/distance", Float64, queue_size=5)
        self.right_dist_pub = rospy.Publisher("right/right_encoder/distance", Float64, queue_size=5)
        self.left_speed_pub = rospy.Publisher("left/left_encoder/speed", Float64, queue_size=5)
        self.right_speed_pub = rospy.Publisher("right/right_encoder/speed", Float64, queue_size=5)
        self.left_command_pub = rospy.Publisher("left/command_speed", Float64, queue_size=5)
        self.right_command_pub = rospy.Publisher("right/command_speed", Float64, queue_size=5)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=5)
        
        # Services
        self.tuning_service = rospy.Service("tune_motor_pid", TuneMotorPID, self.tune_motor_pid)

        # Motor controller objects
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

        # Ultrasonic state tracking objects
        self.num_sensors = len(self.stopping_distances_cm)

        self.sensor_directions = [
            [Direction.FRONT, self.front_sensors],
            [Direction.BACK, self.back_sensors],
            [Direction.LEFT, self.left_sensors],
            [Direction.RIGHT, self.right_sensors]
        ]

        self.trackers_indexed = []
        self.trackers_directed = {
            Direction.FRONT: TrackerCollection(),
            Direction.BACK: TrackerCollection(),
            Direction.LEFT: TrackerCollection(),
            Direction.RIGHT: TrackerCollection(),
        }
        index = 0
        for direction, sensor_indices in self.sensor_directions:
            for sensor_index in sensor_indices:
                stop_dist = stopping_distances_cm[index]
                ease_dist = stop_dist + easing_offset_cm
                tracker = UltrasonicTracker(stop_dist, ease_dist)
                self.trackers_indexed.append(tracker)
                self.trackers_directed[direction].append(tracker)
                index += 1

        # prev state tracking
        self.prev_enc_time = rospy.Time.now()
        self.prev_left_output = 0.0
        self.prev_right_output = 0.0

        # odometry state
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_t = 0.0

        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_vt = 0.0

        # Odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.parent_frame
        self.odom_msg.child_frame_id = self.child_frame

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

        linear_speed_mps, rotational_speed_mps = self.scale_targets(linear_speed_mps, rotational_speed_mps)

        self.left_motor.set_target(linear_speed_mps - rotational_speed_mps)
        self.right_motor.set_target(linear_speed_mps + rotational_speed_mps)

    def left_encoder_callback(self, enc_msg):
        self.left_motor.enc_tick = -enc_msg.data

    def right_encoder_callback(self, enc_msg):
        self.right_motor.enc_tick = enc_msg.data

    def ultrasonic_callback(self, ultrasonic_msg):
        for index, distance in enumerate(ultrasonic_msg.data):
            self.ultrasonic_trackers[index].update(distance)

    def scale_targets(self, lin_vel, ang_vel):
        if lin_vel > 0:
            lin_vel = self.trackers_directed[Direction.FRONT].update(lin_vel)
        elif lin_vel < 0:
            lin_vel = self.trackers_directed[Direction.BACK].update(lin_vel)
        elif ang_vel > 0:
            ang_vel = self.trackers_directed[Direction.LEFT].update(ang_vel)
        elif ang_vel < 0:
            ang_vel = self.trackers_directed[Direction.RIGHT].update(ang_vel)
        return lin_vel, ang_vel

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

            self.compute_odometry(
                self.left_motor.delta_dist,
                self.right_motor.delta_dist,
                self.left_motor.get_speed(),
                self.right_motor.get_speed()
            )

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

        odom_quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, self.odom_t)
        self.tf_broadcaster.sendTransform(
            (self.odom_x, self.odom_y, 0.0),
            odom_quaternion,
            rospy.Time.now(),
            self.child_frame,
            self.parent_frame
        )

        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.pose.pose.position.x = self.odom_x
        self.odom_msg.pose.pose.position.y = self.odom_y
        self.odom_msg.pose.pose.position.z = 0.0

        self.odom_msg.pose.pose.orientation.x = odom_quaternion[0]
        self.odom_msg.pose.pose.orientation.y = odom_quaternion[1]
        self.odom_msg.pose.pose.orientation.z = odom_quaternion[2]
        self.odom_msg.pose.pose.orientation.w = odom_quaternion[3]

        self.odom_msg.twist.twist.linear.x = self.odom_vx
        self.odom_msg.twist.twist.linear.y = self.odom_vy
        self.odom_msg.twist.twist.linear.z = 0.0

        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = self.odom_vt

        self.odom_pub.publish(self.odom_msg)

    def compute_odometry(self, delta_left, delta_right, left_speed, right_speed):
        delta_dist = (delta_right + delta_left) / 2

        # angle = arc / radius
        delta_angle = (delta_right - delta_left) / (self.wheel_distance / 2)

        dx = delta_dist * math.cos(self.odom_t)
        dy = delta_dist * math.sin(self.odom_t)

        self.odom_x += dx
        self.odom_y += dy
        self.odom_t += delta_angle

        speed = (right_speed + left_speed) / 2
        self.odom_vx = speed * math.cos(self.odom_t)
        self.odom_vy = speed * math.sin(self.odom_t)
        self.odom_vt = (right_speed - left_speed) / (self.wheel_distance / 2)

if __name__ == "__main__":
    try:
        node = EarthRoverChassis()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting earth_rover_chassis node")
