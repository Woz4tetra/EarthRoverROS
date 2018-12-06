#ifndef _EARTH_ROVER_LIDAR_GUARD_H_
#define _EARTH_ROVER_LIDAR_GUARD_H_

#include <iostream>
#include <sstream>
#include <math.h>

#include <ros/ros.h>
#include <ros/assert.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <earth_rover_lidar_guard/polygon_checker.h>

using namespace std;

class EarthRoverLidarGuard
{
private:
    ros::NodeHandle nh;

    string laser_topic_name;
    string odom_topic_name;
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;

    ros::Publisher lidar_guard_lock_pub;
    ros::Publisher lidar_guard_pub;

    std_msgs::Bool lidar_guard_lock_msg;
    geometry_msgs::Twist lidar_guard_msg;

    void laser_callback(const sensor_msgs::LaserScan& scan_msg);
    void odom_callback(const nav_msgs::Odometry& odom_msg);

    void set_guard_state(bool state);
    bool is_within_bounds(double range_m, double angle_rad);

    XmlRpc::XmlRpcValue xml_parsed_bounding_polygon;

    double wheel_distance;
    double max_left_speed_mps, max_right_speed_mps;
    double max_linear_speed, max_angular_speed;

    double focus_angle;
    double focus_start_angle;
    double focus_finish_angle;
    bool is_moving;

    bool is_guarded;
    double guard_detection_angle;

    vector<PolygonPoint>* bounding_polygon;

public:
    EarthRoverLidarGuard(ros::NodeHandle* nodehandle);

    int run();
};

#endif  // _EARTH_ROVER_LIDAR_GUARD_H_
