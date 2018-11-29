#!/usr/bin/python
from __future__ import division
import math

import tf
import rospy
from nav_msgs.msg import Odometry

class EarthRoverEKFformatter:
    def __init__(self):
        rospy.init_node(
            "earth_rover_ekf_formatter",
            # log_level=rospy.DEBUG
        )
        self.listener = tf.TransformListener()

        self.odom_msg = Odometry()

        self.odom_msg.header.frame_id = "map"
        self.odom_msg.child_frame_id = "base_link"

        self.odom_pub = rospy.Publisher("gmapping_odom", Odometry, queue_size=5)

    def run(self):
        rospy.sleep(5.0)  # wait for TFs to start
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            try:
                # get latest tf
                # translation expressed as a tuple (x,y,z)
                # rotation quaternion expressed as a tuple (x,y,z,w)
                translation, rotation = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as error:
                rospy.logwarn(str(error))
                continue

            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.pose.pose.position.x = translation[0]
            self.odom_msg.pose.pose.position.y = translation[1]
            self.odom_msg.pose.pose.position.z = translation[2]
            self.odom_msg.pose.pose.orientation.x = rotation[0]
            self.odom_msg.pose.pose.orientation.y = rotation[1]
            self.odom_msg.pose.pose.orientation.z = rotation[2]
            self.odom_msg.pose.pose.orientation.w = rotation[3]
            # self.odom_msg.pose.covariance

            self.odom_pub.publish(self.odom_msg)

            rate.sleep()

if __name__ == "__main__":
    try:
        node = EarthRoverEKFformatter()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting earth_rover_chassis node")
