#include <earth_rover_teensy_bridge/earth_rover_teensy_bridge.h>

#define STR_TO_FLOAT(string)  strtof((string).c_str(), 0)
#define STR_TO_INT(string)  string_to_int64(string)

const string EarthRoverTeensyBridge::NODE_NAME = "earth_rover_teensy_bridge";
const string EarthRoverTeensyBridge::PACKET_END = "\n";

const string EarthRoverTeensyBridge::HELLO_MESSAGE = "hello!" + PACKET_END;
const string EarthRoverTeensyBridge::READY_MESSAGE = "ready!" + PACKET_END;
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

    nh.param<string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh.param<int>("serial_baud", serial_baud, 115200);
    nh.param<string>("act_dist_service_name", act_dist_service_name, "set_guard_distances");
    nh.param<string>("guard_lock_topic", guard_lock_topic, "dist_guard_lock");
    nh.param<string>("guard_topic", guard_topic, "dist_guard");
    nh.param("activation_distances_cm", xml_parsed_activation_distances, xml_parsed_activation_distances);

    ROS_INFO("serial_port: %s", serial_port.c_str());
    ROS_INFO("act_dist_service_name: %s", act_dist_service_name.c_str());
    ROS_INFO("guard_lock_topic: %s", guard_lock_topic.c_str());
    ROS_INFO("guard_topic: %s", guard_topic.c_str());

    guard_lock_pub = nh.advertise<std_msgs::Bool>(guard_lock_topic, 50);
    guard_pub = nh.advertise<geometry_msgs::Twist>(guard_topic, 50);
    act_dist_service = nh.advertiseService(act_dist_service_name, &EarthRoverTeensyBridge::setActivationDists, this);

    activation_distances = new vector<double>();
    ROS_ASSERT_MSG(xml_parsed_activation_distances.size() == 6, "Need 6 activation distances. %d were supplied", xml_parsed_activation_distances.size());
    for (int index = 0; index < xml_parsed_activation_distances.size(); index++)
    {
        double dist = (double)xml_parsed_activation_distances[index];
        activation_distances->push_back(dist);
        ROS_INFO("activation distance for #%d: %fcm", index + 1, activation_distances->at(index));
    }

    ROS_INFO("Earth Rover Teensy bridge init done");
}

bool EarthRoverTeensyBridge::waitForPacket(const string packet)
{
    ros::Time begin = ros::Time::now();
    ros::Duration timeout = ros::Duration(5.0);

    while ((ros::Time::now() - begin) < timeout)
    {
        if (serial_ref.available()) {
            serial_buffer = serial_ref.readline();
            ROS_DEBUG("buffer: %s", serial_buffer.c_str());

            if (serial_buffer.compare(packet) == 0) {
                ROS_INFO("Earth Rover Arduino sent '%s'", packet.substr(0, packet.length() - 1).c_str());
                return true;
            }
        }
    }

    ROS_ERROR("Timeout reached. Serial buffer didn't contain '%s', buffer: %s", packet.c_str(), serial_buffer.c_str());
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

    if (!waitForPacket(HELLO_MESSAGE)) {
        return 1;
    }
    if (!waitForPacket(READY_MESSAGE)) {
        return 1;
    }

    serial_ref.write(START_COMMAND);

    ros::Rate clock_rate(120);  // Hz

    ros::Duration(0.25).sleep();
    writeActivationDists();

    ros::Time prev_time = ros::Time::now();
    while (ros::ok())
    {
        ros::spinOnce();
        clock_rate.sleep();

        if (serial_ref.available())
        {
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

        ros::Time now = ros::Time::now();
        if (now - prev_time > ros::Duration(1.0 / 30.0))  // publish lock at 30Hz
        {
            prev_time = now;
            guard_lock_pub.publish(guard_lock_msg);
            if (guard_lock_msg.data) {  // if is guarded, publish zero velocity commands
                guard_pub.publish(guard_msg);
            }
        }
    }

    serial_ref.write(STOP_COMMAND);

    return 0;
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
            if (!guard_lock_msg.data && activated_sensor != 0)  {
                activation_time = ros::Time::now();
                guard_lock_msg.data = true;
            }
            break;
        case 'd':
            activated_distance = STR_TO_FLOAT(token.substr(1));
            break;
        default:
            ROS_WARN("Invalid segment type for earth rover arduino bridge! Segment: '%c', packet: '%s'", serial_buffer.at(0), serial_buffer.c_str());
            break;
    }
}

void EarthRoverTeensyBridge::writeActivationDists()
{
    for (size_t index = 0; index < activation_distances->size(); index++) {
        std::ostringstream command_stream;
        command_stream << "d" << index << "\t" << activation_distances->at(index) << "\n";

        serial_ref.write(command_stream.str());
    }
}

bool EarthRoverTeensyBridge::setActivationDists(ActivationDistances::Request &req, ActivationDistances::Response &res)
{
    // if activation distance is negative, skip setting it.
    if (req.dist1 >= 0.0) (*activation_distances)[0] = req.dist1;
    if (req.dist2 >= 0.0) (*activation_distances)[1] = req.dist2;
    if (req.dist3 >= 0.0) (*activation_distances)[2] = req.dist3;
    if (req.dist4 >= 0.0) (*activation_distances)[3] = req.dist4;
    if (req.dist5 >= 0.0) (*activation_distances)[4] = req.dist5;
    if (req.dist6 >= 0.0) (*activation_distances)[5] = req.dist6;

    writeActivationDists();

    return true;
}
