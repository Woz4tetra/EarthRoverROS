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

const string EarthRoverTeensyBridge::ULTRASONIC_MESSAGE_HEADER = "u";

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
    nh.param<string>("ultrasonic_topic_name", ultrasonic_topic_name, "ultrasonic");

    ultrasonic_pub = nh.advertise<std_msgs::Float32MultiArray>(ultrasonic_topic_name, 50);
    ultrasonic_msg.data.clear();

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
            if (*serial_buffer.rbegin() == '\n') {

                if (serial_buffer.compare(response_packet) == 0) {
                    ROS_INFO(
                        "%s sent '%s'",
                        serial_port.c_str(), response_packet.substr(0, response_packet.length() - 1).c_str()
                    );
                    return true;
                }

                serial_buffer = "";
            }
            // serial_buffer = serial_ref.readline();
        }

        if (now - prev_write_time > ros::Duration(2.0)) {
            prev_write_time = now;

            serial_ref.write(ask_packet);
            ROS_INFO("Earth Rover sending ask packet '%s' again to %s", ask_packet.c_str(), serial_port.c_str());
        }

        now = ros::Time::now();
        ros::Duration(0.005).sleep();  // give the CPU a break
    }

    ROS_ERROR(
        "Timeout reached. Serial buffer didn't contain '%s', buffer: %s for port %s",
        response_packet.c_str(), serial_buffer.c_str(), serial_port.c_str()
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

    ros::Time prev_msg_time = ros::Time::now();
    int prev_activated_sensor = -1;

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
            // ROS_DEBUG("buffer: %s", serial_buffer.c_str());

            if (serial_buffer.at(0) == '-') {
                ROS_WARN("message: %s", serial_buffer.substr(1).c_str());
                continue;
            }

            // ROS_DEBUG("buffer: %s", serial_buffer.c_str());

            // Parse encoder segment
            if (serial_buffer.length() > ULTRASONIC_MESSAGE_HEADER.size() &&
                serial_buffer.compare(0, ULTRASONIC_MESSAGE_HEADER.size(), ULTRASONIC_MESSAGE_HEADER) == 0) {
                parseActDistMessage();

                ultrasonic_pub.publish(ultrasonic_msg);
            }
        }
    }

    serial_ref.write(STOP_COMMAND);

    return 0;
}

void EarthRoverTeensyBridge::parseActDistMessage()
{
    size_t start_index = ULTRASONIC_MESSAGE_HEADER.size() + 1;
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

void EarthRoverTeensyBridge::setVectorLength(size_t length) {
    ROS_INFO("resizing ultrasonic array, %lu", length);
    ultrasonic_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    ultrasonic_msg.layout.dim[0].label = "distances";
    ultrasonic_msg.layout.dim[0].size = length;
    ultrasonic_msg.layout.dim[0].stride = length;
    ultrasonic_msg.layout.data_offset = 0;
    ultrasonic_msg.data.resize(length, 0.0);
}

void EarthRoverTeensyBridge::assignDist(size_t index, double dist) {
    if (index < ultrasonic_msg.data.size()) {
        ultrasonic_msg.data[index] = dist;
    }
    else {
        ROS_ERROR("Assigning distance out of range of message. Message length: %lu. Index requested: %lu", ultrasonic_msg.data.size(), index);
    }
}

void EarthRoverTeensyBridge::parseActDistToken(string token)
{
    switch (token.at(0)) {
        // case 't': ROS_DEBUG("earth rover teensy time: %s", token.substr(1).c_str()); break;
        case 't': break;
        case 'l': setVectorLength(STR_TO_INT(token.substr(1))); break;
        case 'd': assignDist(STR_TO_INT(token.substr(1, 2)), STR_TO_FLOAT(token.substr(3))); break;
        default:
            ROS_WARN(
                "Invalid segment type for earth rover teensy bridge! Segment: '%c', packet: '%s'",
                serial_buffer.at(0), serial_buffer.c_str()
            );
            break;
    }
}
