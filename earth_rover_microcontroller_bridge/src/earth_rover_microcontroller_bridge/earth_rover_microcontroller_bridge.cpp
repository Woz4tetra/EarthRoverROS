#include <earth_rover_microcontroller_bridge/earth_rover_microcontroller_bridge.h>

#define STR_TO_FLOAT(string)  strtof((string).c_str(), 0)
#define STR_TO_INT(string)  string_to_int64(string)

const string EarthRoverMicroControllerBridge::NODE_NAME = "earth_rover_microcontroller_bridge";
const string EarthRoverMicroControllerBridge::PACKET_END = "\n";

const string EarthRoverMicroControllerBridge::HELLO_MESSAGE = "hello!" + PACKET_END;
const string EarthRoverMicroControllerBridge::READY_MESSAGE = "ready!" + PACKET_END;
const string EarthRoverMicroControllerBridge::READY_ASK_COMMAND = "r" + PACKET_END;
const string EarthRoverMicroControllerBridge::START_COMMAND = "g" + PACKET_END;
const string EarthRoverMicroControllerBridge::STOP_COMMAND = "s" + PACKET_END;
const string EarthRoverMicroControllerBridge::MESSAGE_DELIMITER = "\t";

const string EarthRoverMicroControllerBridge::ENCODER_MESSAGE_HEADER = "enc";

long long string_to_int64(string s) {
    stringstream ss(s);
    long long integer = 0;
    ss >> integer;
    return integer;
}

EarthRoverMicroControllerBridge::EarthRoverMicroControllerBridge(ros::NodeHandle* nodehandle):
    nh(*nodehandle)
{
    ROS_INFO("Earth Rover Arduino bridge starting...");

    nh.param<string>("enc_pub_topic", enc_pub_topic, "ticks");
    nh.param<string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh.param<int>("serial_baud", serial_baud, 115200);
    nh.param<string>("led_control_service_name", led_control_service_name, "led_control_left");
    nh.param<string>("led_control_start_command", led_control_start_command, "-");

    ROS_INFO("enc_pub_topic: %s", enc_pub_topic.c_str());
    ROS_INFO("serial_port: %s", serial_port.c_str());
    ROS_INFO("led_control_service_name: %s", led_control_service_name.c_str());
    ROS_INFO("led_control_start_command: %s", led_control_start_command.c_str());

    led_control_start_command += "\n";

    encoder_pub = nh.advertise<std_msgs::Int64>(enc_pub_topic, 50);
    led_control_service = nh.advertiseService(led_control_service_name, &EarthRoverMicroControllerBridge::controlLeds, this);

    ROS_INFO("Earth Rover Arduino bridge init done");
}

bool EarthRoverMicroControllerBridge::waitForPacket(const string ask_packet, const string response_packet)
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
                // ROS_DEBUG("buffer: %s", serial_buffer.c_str());

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
            now = prev_write_time;

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

int EarthRoverMicroControllerBridge::run()
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

    ros::Duration(0.25).sleep();
    if (!waitForPacket(READY_ASK_COMMAND, READY_MESSAGE)) {
        return 1;
    }

    serial_ref.write(START_COMMAND);

    ros::Rate clock_rate(120);  // Hz

    // ros::Duration(0.25).sleep();
    ROS_INFO("led_control_start_command: %s", led_control_start_command.c_str());
    serial_ref.write(led_control_start_command);

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

            // ROS_DEBUG("buffer: %s", serial_buffer.c_str());

            // Parse encoder segment
            if (serial_buffer.length() > ENCODER_MESSAGE_HEADER.size() &&
                serial_buffer.compare(0, ENCODER_MESSAGE_HEADER.size(), ENCODER_MESSAGE_HEADER) == 0) {
                parseEncoderMessage();
            }
        }
    }

    serial_ref.write(STOP_COMMAND);

    return 0;
}

void EarthRoverMicroControllerBridge::parseEncoderMessage()
{
    size_t start_index = ENCODER_MESSAGE_HEADER.size() + 1;
    serial_buffer = serial_buffer.substr(start_index, serial_buffer.size() - start_index - 1);

    size_t pos = 0;
    string token;
    while ((pos = serial_buffer.find(MESSAGE_DELIMITER)) != string::npos)
    {
        token = serial_buffer.substr(0, pos);
        if (token.size() == 0) {
            break;
        }
        parseToken(token);
        serial_buffer.erase(0, pos + MESSAGE_DELIMITER.length());
    }

    parseToken(serial_buffer);  // remaining buffer is the last token (has newline removed)
}

void EarthRoverMicroControllerBridge::parseToken(string token)
{
    switch (token.at(0)) {
        // case 't': ROS_DEBUG("earth rover arduino time: %s", token.substr(1).c_str()); break;
        case 't': break;
        case 'p':
            encoder_msg.data = STR_TO_INT(token.substr(1));
            // ROS_DEBUG("encoder_msg: %li", encoder_msg.data);

            encoder_pub.publish(encoder_msg);
            break;
        default:
            ROS_WARN("Invalid segment type for earth rover arduino bridge! Segment: '%c', packet: '%s'", serial_buffer.at(0), serial_buffer.c_str());
            break;
    }
}

bool EarthRoverMicroControllerBridge::controlLeds(LedControl::Request &req, LedControl::Response &res)
{
    string command = req.command;
    command += "\n";
    serial_ref.write(command);
    return true;
}
