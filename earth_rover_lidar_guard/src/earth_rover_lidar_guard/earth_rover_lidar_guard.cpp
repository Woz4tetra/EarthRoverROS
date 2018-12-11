#include <earth_rover_lidar_guard/earth_rover_lidar_guard.h>

EarthRoverLidarGuard::EarthRoverLidarGuard(ros::NodeHandle* nodehandle):
nh(*nodehandle)
{
    nh.param<string>("laser_topic_name", laser_topic_name, "scan");
    nh.param<string>("odom_topic_name", odom_topic_name, "odom");
    nh.param<double>("max_left_speed_mps", max_left_speed_mps, 1.0);
    nh.param<double>("max_right_speed_mps", max_right_speed_mps, 1.0);
    nh.param<double>("wheel_distance", wheel_distance, 1.0);
    nh.param<double>("angle_tf_deg", angle_tf, 0.0);
    nh.param("bounding_polygon", xml_parsed_bounding_polygon, xml_parsed_bounding_polygon);

    angle_tf *= M_PI / 180.0;

    laser_sub = nh.subscribe(laser_topic_name, 5, &EarthRoverLidarGuard::laser_callback, this);
    odom_sub = nh.subscribe(odom_topic_name, 5, &EarthRoverLidarGuard::odom_callback, this);

    lidar_guard_lock_pub = nh.advertise<std_msgs::Bool>("lidar_guard_lock", 5);
    lidar_guard_pub = nh.advertise<geometry_msgs::Twist>("lidar_guard", 5);

    focus_angle = 0.0;
    focus_start_angle = -M_PI;
    focus_finish_angle = M_PI;
    is_moving = false;

    is_guarded = false;
    guard_detection_angle = 0.0;

    bounding_polygon = new vector<double>();

    // for this calculation, both motors are rotating forwards
    max_linear_speed = (max_right_speed_mps + max_left_speed_mps) / 2;

    // for this calculation, the left motor is rotating backwards at max speed
    max_angular_speed = (max_right_speed_mps + max_left_speed_mps) / (wheel_distance / 2);

    ROS_ASSERT_MSG(!(xml_parsed_bounding_polygon.size() & 1), "An odd number of numbers was supplied for the bounding polygon. There must an even number for x, y pairs.");
    ROS_ASSERT_MSG(xml_parsed_bounding_polygon.size() > 4, "Supplied points does not form a polygon. Length is %d", xml_parsed_bounding_polygon.size());
    for (int index = 0; index < xml_parsed_bounding_polygon.size(); index += 2)
    {
        double x = (double)xml_parsed_bounding_polygon[index];
        double y = (double)xml_parsed_bounding_polygon[index + 1];
        bounding_polygon->push_back(x);
        bounding_polygon->push_back(y);
        ROS_INFO("point %d of %d; x: %0.4f, y: %0.4f", index, xml_parsed_bounding_polygon.size(), x, y);

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
                ROS_WARN("LIDAR detected an obstacle at angle %0.2f deg", guard_detection_angle);
                prev_time = now;
            }
        }

        lidar_guard_lock_pub.publish(lidar_guard_lock_msg);
        ros::spinOnce();
        clock_rate.sleep();
    }

    return 0;
}

bool EarthRoverLidarGuard::is_within_bounds(double range_m, double angle_rad)
{
    if (isinf(range_m)) {
        return false;
    }
    double x = range_m * cos(angle_rad + angle_tf);
    double y = range_m * sin(angle_rad + angle_tf);

    return isInside(bounding_polygon, x, y);
}

void EarthRoverLidarGuard::set_guard_state(bool state) {
    is_guarded = state;
    lidar_guard_lock_msg.data = state;
}

void EarthRoverLidarGuard::laser_callback(const sensor_msgs::LaserScan& scan_msg)
{
    // assumes min angle < 0 and zero degrees is forward
    if (!is_moving) {
        set_guard_state(false);
        return;
    }

    double ray_angle = scan_msg.angle_min;

    // prevents odom_callback from overriding these values in the middle of the loop.
    double local_start_angle = focus_start_angle;
    double local_finish_angle = focus_finish_angle;

    for (size_t index = 0; index < scan_msg.ranges.size(); index++) {
        if (local_start_angle < ray_angle && ray_angle < local_finish_angle) {
            // consider laser point for safety check
            if (is_within_bounds(scan_msg.ranges[index], ray_angle)) {
                // throw stop signal to speed command mux

                guard_detection_angle = ray_angle;
                set_guard_state(true);
                ROS_WARN("LIDAR guard thrown! Detected an obstacle %0.2fm away at angle %0.2f deg in range (%0.2f...%0.2f)",
                    scan_msg.ranges[index], guard_detection_angle * 180.0 / M_PI, local_start_angle * 180.0 / M_PI, local_finish_angle * 180.0 / M_PI);

                return;
            }
        }
        ray_angle += scan_msg.angle_increment;
    }
    set_guard_state(false);
}

void EarthRoverLidarGuard::odom_callback(const nav_msgs::Odometry& odom_msg)
{
    double vx = odom_msg.twist.twist.linear.x;
    // double vy = odom_msg.twist.twist.linear.y;
    double angular_speed = odom_msg.twist.twist.angular.z;

    if (fabs(vx) > 0.07 || fabs(angular_speed) > 0.15) {
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
    rotation_mat.getEulerYPR(yaw, pitch, roll, 1);

    // double linear_speed = sqrt(vx * vx + vy * vy);  // linear speed without +/- direction
    double linear_velocity = vx / cos(yaw); // linear velocity
    double linear_percent = linear_velocity / max_linear_speed;
    double angular_percent = angular_speed / max_angular_speed;

    focus_angle = atan2(angular_percent, linear_percent);
    focus_start_angle = focus_angle - M_PI / 2;
    focus_finish_angle = focus_angle + M_PI / 2;

    if (focus_start_angle < -M_PI) {
        focus_start_angle += 2 * M_PI;
    }
    if (focus_finish_angle < -M_PI) {
        focus_finish_angle += 2 * M_PI;
    }

    if (focus_start_angle > focus_finish_angle) {
        double tmp = focus_start_angle;
        focus_start_angle = focus_finish_angle;
        focus_finish_angle = tmp;
    }

    ROS_INFO("focus angle: %0.2f", focus_angle * 180.0 / M_PI);
}
