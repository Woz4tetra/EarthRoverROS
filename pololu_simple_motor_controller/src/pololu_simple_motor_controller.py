#!/usr/bin/python
from __future__ import division

from smc import SMC

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int16

class PololuSimpleMotorController:
    def __init__(self, ):
        rospy.init_node(
            "pololu_simple_motor_controller",
            # log_level=rospy.DEBUG
        )
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to pololu simple motor controller")
        self.port = rospy.get_param("~serial_port", "/dev/ttyUSB0")
        self.baud = rospy.get_param("~serial_baud", 115200)
        self.speed_topic_name = rospy.get_param("~speed_topic_name", "vel")
        self.brake_topic_name = rospy.get_param("~brake_topic_name", "brake")
        self.reverse_motor = rospy.get_param("~reverse_motor", False)

        rospy.loginfo("pololu simple motor controller serial_port: '%s'", self.port)
        rospy.loginfo("pololu simple motor controller serial_baud: '%s'", self.baud)
        if self.reverse_motor:
            rospy.loginfo("reversing motor direction for pololu simple motor controller")
        self.motor_controller = SMC(self.port, self.baud)
        self.motor_controller.init()

        self.speed_sub = rospy.Subscriber(self.speed_topic_name, Float64, self.speed_command_callback, queue_size=1)
        self.brake_sub = rospy.Subscriber(self.brake_topic_name, Int16, self.brake_command_callback, queue_size=1)

        self.motor_controller.speed(0)

    def shutdown(self):
        self.speed_sub.unregister()
        self.brake_sub.unregister()
        # self.motor_controller.stop()
        # self.motor_controller.close()

    def speed_command_callback(self, command):
        motor_command = max(-1.0, min(command.data, 1.0))
        motor_command = int(motor_command * 3200)
        if self.reverse_motor:
            motor_command = -motor_command
        rospy.logdebug("Sending: %s" % motor_command)
        self.motor_controller.speed(motor_command)

    def brake_command_callback(self, command):
        self.motor_controller.mbreak(int(command.data))


if __name__ == "__main__":
    try:
        PololuSimpleMotorController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting pololu_simple_motor_controller node")
