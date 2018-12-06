#include <earth_rover_lidar_guard/earth_rover_lidar_guard.h>
#include <earth_rover_lidar_guard/polygon_checker.h>

EarthRoverLidarGuard::EarthRoverLidarGuard(ros::NodeHandle* nodehandle):
nh(*nodehandle)
{
    nh.param<string>("laser_topic_name", laser_topic_name, "scan");
    nh.param<string>("odom_topic_name", odom_topic_name, "odom");
    nh.param<double>("max_linear_speed", max_linear_speed, 1.0);
    nh.param<double>("max_angular_speed", max_angular_speed, 1.0);
    nh.param("bounding_polygon", xml_parsed_bounding_polygon, xml_parsed_bounding_polygon);

    laser_sub = nh.subscribe(laser_topic_name, 5, &EarthRoverLidarGuard::laser_callback, this);
    odom_sub = nh.subscribe(odom_topic_name, 5, &EarthRoverLidarGuard::odom_callback, this);

    lidar_guard_lock_pub = nh.advertise("lidar_guard_lock", std_msgs::Bool, queue_size=5);
    lidar_guard_pub = nh.advertise("lidar_guard", geometry_msgs::Twist, queue_size=5);

    focus_angle = 0.0;
    focus_start_angle = -M_PI;
    focus_finish_angle = M_PI;
    is_moving = false;

    is_guarded = false;
    guard_detection_angle = 0.0;

    ROS_ASSERT_MSG(!(xml_parsed_bounding_polygon.size() & 1), "An odd number of numbers was supplied for the bounding polygon. There must an even number for x, y pairs.");
    ROS_ASSERT_MSG(xml_parsed_bounding_polygon.size() > 4, "Supplied points does not form a polygon. Length is %d", xml_parsed_bounding_polygon.size());
    for (int index = 0; index < xml_parsed_bounding_polygon.size(); index += 2)
    {
        Point p = {
            (double)xml_parsed_bounding_polygon[i],
            (double)xml_parsed_bounding_polygon[i + 1]
        };
        bounding_polygon.push_back(p);
    }
}

int EarthRoverLidarGuard::run()
{
    ros::Rate clock_rate(30);  // Hz

    ros::Time prev_time = ros::Time::now();
    ros::Time now = ros::Time::now();
    while (ros::ok())
    {
        if (is_guarded) {
            lidar_guard_pub.publish(lidar_guard_msg);

            now = ros::Time::now();
            if (now - prev_time > ros::Duration(1.0)) {
                ROS_WARN("LIDAR detected an obstacle at angle %0.2f", guard_detection_angle);
                prev_time = now;
            }
        }

        lidar_guard_pub.publish(lidar_guard_lock_msg);
        ros::spinOnce();
        clock_rate.sleep();
    }

    return 0;
}

bool EarthRoverLidarGuard::is_within_bounds(double range_m, double angle_rad)
{
    double x = range_m * cos(angle_rad);
    double y = range_m * sin(angle_rad);

    return isInside(bounding_polygon, x, y);
}

void EarthRoverLidarGuard::laser_callback(const sensor_msgs::LaserScan& scan_msg)
{
    // assumes min angle < 0
    if (!is_moving) {
        lidar_guard_lock_msg.data = false;
        return;
    }

    double ray_angle = scan_msg.angle_min;
    for (size_t index = 0; index < scan_msg.ranges.size(); index++) {
        // if within the selected range
        if (focus_start_angle < ray_angle && ray_angle < focus_finish_angle) {
            // consider laser point for safety check
            if (is_within_bounds(scan_msg.ranges[index], ray_angle)) {
                // throw stop signal to speed command mux
                guard_detection_angle = ray_angle;
                lidar_guard_lock_msg.data = true;
                ROS_WARN("LIDAR guard thrown! Detected an obstacle at angle %0.2f", guard_detection_angle);

                return;
            }
        }

        ray_angle += scan_msg.angle_increment;
    }
    lidar_guard_lock_msg.data = false;
}

void EarthRoverLidarGuard::odom_callback(const nav_msgs::Odometry& odom_msg)
{
    double vx = odom_msg.twist.twist.linear.x;
    // double vy = odom_msg.twist.twist.linear.y;
    double angular_speed = odom_msg.twist.twist.angular.z;

    if (fabs(vx) > 0.0 && fabs(angular_speed) > 0.0) {
        is_moving = true;
    }
    else {
        is_moving = false;
        return;
    }

    const tf::Quaternion q(
        odom_msg.pose.pose.orientation.w,
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z
    );
    const tf::Matrix3x3 rotation_mat(q);
    tfScalar yaw, pitch, roll;
    rotation_mat.getEulerYPR(&yaw, &pitch, &roll);

    // double linear_speed = sqrt(vx * vx + vy * vy);  // linear speed without +/- direction
    double linear_velocity = vx / cos(yaw); // linear velocity
    double linear_percent = linear_velocity / max_linear_speed;
    double angular_percent = angular_speed / max_angular_speed;

    focus_angle = atan2(linear_percent, angular_percent);
    focus_start_angle = focus_angle - M_PI;
    focus_finish_angle = focus_angle + M_PI;

    ROS_INFO("focus angle: %0.2f", focus_angle * 180.0 / M_PI);
}
