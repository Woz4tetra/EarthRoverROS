#include <earth_rover_teensy_bridge/earth_rover_teensy_bridge.h>

#define STR_TO_FLOAT(string)  strtof((string).c_str(), 0)
#define STR_TO_INT(string)  string_to_int64(string)

const string EarthRoverTeensyBridge::NODE_NAME = "earth_rover_teensy_bridge";
const string EarthRoverTeensyBridge::PACKET_END = "\n";

const string EarthRoverTeensyBridge::HELLO_MESSAGE = "hello!" + PACKET_END;
const string EarthRoverTeensyBridge::READY_MESSAGE = "ready!" + PACKET_END;
const string EarthRoverTeensyBridge::READY_ASK_COMMAND = "r" + PACKET_END;
const string EarthRoverTeensyBridge::START_COMMAND = "g" + PACKET_END;
const string EarthRoverTeensyBridge::STOP_COMMAND = "s" + PACKET_END;
const string EarthRoverTeensyBridge::MESSAGE_DELIMITER = "\t";

const string EarthRoverTeensyBridge::ACTIVATE_MESSAGE_HEADER = "a";

long long string_to_int64(string s) {
    stringstream ss(s);
    long long integer = 0;
    ss >> integer;
    return integer;
}

EarthRoverTeensyBridge::EarthRoverTeensyBridge(ros::NodeHandle* nodehandle):
    nh(*nodehandle)
{
    ROS_INFO("Earth Rover Teensy bridge starting...");

    double activation_timeout_double = 0.0;
    nh.param<string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh.param<int>("serial_baud", serial_baud, 115200);
    nh.param<string>("act_dist_service_name", act_dist_service_name, "set_guard_distances");
    nh.param<string>("guard_lock_topic", guard_lock_topic, "dist_guard_lock");
    nh.param<string>("guard_topic", guard_topic, "dist_guard");
    nh.param<string>("odom_topic_name", odom_topic_name, "odom");
    nh.param<double>("activation_timeout", activation_timeout_double, 0.5);
    nh.param<double>("min_angular_activation_speed", min_angular_activation_speed, 0.15);
    nh.param<double>("min_linear_activation_speed", min_linear_activation_speed, 0.07);
    nh.param("activation_distances_cm", xml_parsed_activation_distances, xml_parsed_activation_distances);

    // ROS_INFO("serial_port: %s", serial_port.c_str());
    // ROS_INFO("act_dist_service_name: %s", act_dist_service_name.c_str());
    // ROS_INFO("guard_lock_topic: %s", guard_lock_topic.c_str());
    // ROS_INFO("guard_topic: %s", guard_topic.c_str());

    guard_lock_pub = nh.advertise<std_msgs::Bool>(guard_lock_topic, 50);
    guard_pub = nh.advertise<geometry_msgs::Twist>(guard_topic, 50);
    act_dist_service = nh.advertiseService(act_dist_service_name, &EarthRoverTeensyBridge::setActivationDists, this);

    odom_sub = nh.subscribe(odom_topic_name, 5, &EarthRoverTeensyBridge::odom_callback, this);

    guard_lock_msg.data = false;
    activated_sensor = 0;
    activated_dist = 0.0;

    is_moving = false;
    odom_angular_speed = 0.0;
    odom_linear_velocity = 0.0;

    sensor_states = new vector<SensorStateMachine>();
    ROS_ASSERT_MSG(xml_parsed_activation_distances.size() == 6, "Need 6 activation distances. %d were supplied", xml_parsed_activation_distances.size());
    for (int index = 0; index < xml_parsed_activation_distances.size(); index++)
    {
        double dist = (double)xml_parsed_activation_distances[index];
        SensorStateMachine s(dist, activation_timeout_double);

        sensor_states->push_back(s);
        ROS_INFO("activation distance for #%d: %fcm", index + 1, dist);
    }

    ROS_INFO("Earth Rover Teensy bridge init done");
}

bool EarthRoverTeensyBridge::waitForPacket(const string ask_packet, const string response_packet)
{
    serial_ref.write(ask_packet);
    ros::Duration(0.01).sleep();

    ros::Time begin = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Time prev_write_time = ros::Time::now();

    serial_buffer = "";

    while ((now - begin) < ros::Duration(10.0))
    {
        if (serial_ref.available())
        {
            serial_buffer += serial_ref.read(1);
            if (serial_buffer.back() == '\n') {
                ROS_DEBUG("buffer: %s", serial_buffer.c_str());

                if (serial_buffer.compare(response_packet) == 0) {
                    ROS_INFO(
                        "Earth Rover Teensy sent '%s'",
                        response_packet.substr(0, response_packet.length() - 1).c_str()
                    );
                    return true;
                }

                serial_buffer = "";
            }
            // serial_buffer = serial_ref.readline();
        }

        if (now - prev_write_time > ros::Duration(2.0)) {
            now = prev_write_time;

            serial_ref.write(ask_packet);
            ROS_INFO("Earth Rover Teensy sending ask packet '%s' again", ask_packet);
        }

        now = ros::Time::now();
        ros::Duration(0.005).sleep();  // give the CPU a break
    }

    ROS_ERROR(
        "Timeout reached. Serial buffer didn't contain '%s', buffer: %s",
        packet.c_str(), serial_buffer.c_str()
    );
    return false;
}

int EarthRoverTeensyBridge::run()
{
    try
    {
        serial_ref.setPort(serial_port);
        serial_ref.setBaudrate(serial_baud);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_ref.setTimeout(timeout);
        serial_ref.open();
    }
    catch (serial::IOException e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        ROS_ERROR_STREAM(serial_port);
        return -1;
    }

    ros::Rate clock_rate(120);  // Hz, must be faster than microcontroller's write rate

    ros::Duration(0.25).sleep();
    if (!waitForPacket(READY_ASK_COMMAND, READY_MESSAGE)) {
        return 1;
    }

    serial_ref.write(START_COMMAND);

    writeActivationDists();

    ros::Time prev_msg_time = ros::Time::now();
    ros::Time now = ros::Time::now();
    while (ros::ok())
    {
        ros::spinOnce();
        clock_rate.sleep();

        if (serial_ref.available())
        {
            // this sets the loop speed also since it only proceeds when a new line is found.
            // a new line is printed when a timer expires on the teensy
            serial_buffer = serial_ref.readline();

            if (serial_buffer.at(0) == '-') {
                ROS_WARN("message: %s", serial_buffer.substr(1).c_str());
                continue;
            }

            ROS_DEBUG("buffer: %s", serial_buffer.c_str());

            // Parse encoder segment
            if (serial_buffer.length() > ACTIVATE_MESSAGE_HEADER.size() &&
                serial_buffer.compare(0, ACTIVATE_MESSAGE_HEADER.size(), ACTIVATE_MESSAGE_HEADER) == 0) {
                parseActDistMessage();
            }
        }

        checkGuardState();

        now = ros::Time::now();
        if (activated_sensor != 0 && now - prev_msg_time > ros::Duration(0.25)) {
            ROS_INFO("Sensor #%d activated with dist %0.3f", activated_sensor, activated_dist);
            prev_msg_time = now;
        }
    }

    serial_ref.write(STOP_COMMAND);

    return 0;
}

void EarthRoverTeensyBridge::checkVelocityDirection()
{
    return (
        // if front sensors activated and robot is moving forward
        ((activated_sensor == 1 || activated_sensor == 2) && odom_linear_velocity > 0.0) ||

        // if right side sensor activated and robot is rotating right
        (activated_sensor == 3 && odom_angular_speed < 0.0) ||

        // if left side sensor activated and robot is rotating left
        (activated_sensor == 4 && odom_angular_speed > 0.0) ||

        // if back sensors activated and robot is moving backward
        ((activated_sensor == 5 || activated_sensor == 6) && odom_linear_velocity < 0.0)
    );
}

void EarthRoverTeensyBridge::checkGuardState()
{
    // for (size_t i = 0; i < sensor_states->size(); i++) {
    //     if (activated_sensor == i + 1) {
    //         guard_lock_msg.data = sensor_states->at(i).update(activated_dist);
    //     }
    //     else {
    //         sensor_states->at(i).set_to_waiting();
    //     }
    // }
    if (activated_sensor == 0) {
        guard_lock_msg.data = false;
    }
    else if (checkVelocityDirection())
    {
        // update distance sensor state machine and determine whether to throw the guard
        guard_lock_msg.data = sensor_states->at(activated_sensor - 1).update(activated_dist);
    }

    guard_lock_pub.publish(guard_lock_msg);

    if (guard_lock_msg.data) {  // if is guarded, publish zero velocity commands
        guard_pub.publish(guard_msg);
    }
}


void EarthRoverTeensyBridge::parseActDistMessage()
{
    size_t start_index = ACTIVATE_MESSAGE_HEADER.size() + 1;
    serial_buffer = serial_buffer.substr(start_index, serial_buffer.size() - start_index - 1);

    size_t pos = 0;
    string token;
    while ((pos = serial_buffer.find(MESSAGE_DELIMITER)) != string::npos)
    {
        token = serial_buffer.substr(0, pos);
        if (token.size() == 0) {
            break;
        }
        parseActDistToken(token);
        serial_buffer.erase(0, pos + MESSAGE_DELIMITER.length());
    }

    parseActDistToken(serial_buffer);  // remaining buffer is the last token (has newline removed)
}

void EarthRoverTeensyBridge::parseActDistToken(string token)
{
    switch (token.at(0)) {
        case 't': ROS_DEBUG("earth rover teensy time: %s", token.substr(1).c_str()); break;
        case 'n':
            activated_sensor = STR_TO_INT(token.substr(1));
            break;
        case 'd':
            activated_dist = STR_TO_FLOAT(token.substr(1));
            break;
        default:
            ROS_WARN(
                "Invalid segment type for earth rover teensy bridge! Segment: '%c', packet: '%s'",
                serial_buffer.at(0), serial_buffer.c_str()
            );
            break;
    }
}

void EarthRoverTeensyBridge::odom_callback(const nav_msgs::Odometry& odom_msg)
{
    double vx = odom_msg.twist.twist.linear.x;
    odom_angular_speed = odom_msg.twist.twist.angular.z;

    if (fabs(angular_speed) <= min_angular_activation_speed) {
        odom_angular_speed = 0.0;
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
    odom_linear_velocity = vx / cos(yaw); // linear velocity

    if (fabs(odom_linear_velocity) <= min_linear_activation_speed) {
        odom_linear_velocity = 0.0;
    }
}

void EarthRoverTeensyBridge::writeActivationDists()
{
    for (size_t index = 0; index < sensor_states->size(); index++) {
        // format message to look like d0\t0.000\n  d[sensor index]\t[activation distance]\n
        double dist = sensor_states->at(index).get_act_dist();
        serial_ref.write(boost::str(boost::format("d%d\t%0.3f\n") % (index + 1) % dist));
    }
}

bool EarthRoverTeensyBridge::setActivationDists(ActivationDistances::Request &req, ActivationDistances::Response &res)
{
    // if activation distance is negative, skip setting it.
    if (req.dist1 >= 0.0) sensor_states->at(0).set_act_dist(req.dist1);
    if (req.dist2 >= 0.0) sensor_states->at(1).set_act_dist(req.dist2);
    if (req.dist3 >= 0.0) sensor_states->at(2).set_act_dist(req.dist3);
    if (req.dist4 >= 0.0) sensor_states->at(3).set_act_dist(req.dist4);
    if (req.dist5 >= 0.0) sensor_states->at(4).set_act_dist(req.dist5);
    if (req.dist6 >= 0.0) sensor_states->at(5).set_act_dist(req.dist6);

    writeActivationDists();

    return true;
}
