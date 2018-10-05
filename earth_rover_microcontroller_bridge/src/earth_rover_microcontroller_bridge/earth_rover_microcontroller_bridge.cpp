#include <earth_rover_microcontroller_bridge/earth_rover_microcontroller_bridge.h>

#define STR_TO_FLOAT(string)  strtof((string).c_str(), 0)
#define STR_TO_INT(string)  string_to_int64(string)

const string EarthRoverMicroControllerBridge::NODE_NAME = "earth_rover_microcontroller_bridge";
const string EarthRoverMicroControllerBridge::PACKET_END = "\n";

const string EarthRoverMicroControllerBridge::HELLO_MESSAGE = "hello!" + PACKET_END;
const string EarthRoverMicroControllerBridge::READY_MESSAGE = "ready!" + PACKET_END;
const string EarthRoverMicroControllerBridge::START_COMMAND = "g" + PACKET_END;
const string EarthRoverMicroControllerBridge::STOP_COMMAND = "s" + PACKET_END;
const string EarthRoverMicroControllerBridge::MESSAGE_DELIMITER = "\t";

const string EarthRoverMicroControllerBridge::ENCODER_MESSAGE_HEADER = "e";

const size_t EarthRoverMicroControllerBridge::MOTOR_COMMAND_MESSAGE_LEN = 7;

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

    nh.param<string>("left_enc_pub_topic", left_enc_pub_topic, "/left_encoder");
    nh.param<string>("right_enc_pub_topic", right_enc_pub_topic, "/right_encoder");
    nh.param<string>("left_motor_sub_topic", left_motor_sub_topic, "/left_motor_commands");
    nh.param<string>("right_motor_sub_topic", right_motor_sub_topic, "/right_motor_commands");
    nh.param<string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh.param<int>("serial_baud", serial_baud, 115200);

    ROS_INFO("left_enc_pub_topic: %s", left_enc_pub_topic.c_str());
    ROS_INFO("right_enc_pub_topic: %s", right_enc_pub_topic.c_str());
    ROS_INFO("left_motor_sub_topic: %s", left_motor_sub_topic.c_str());
    ROS_INFO("right_motor_sub_topic: %s", right_motor_sub_topic.c_str());
    ROS_INFO("serial_port: %s", serial_port.c_str());

    right_encoder_pub = nh.advertise<std_msgs::Int64>(right_enc_pub_topic, 5);
    left_encoder_pub = nh.advertise<std_msgs::Int64>(left_enc_pub_topic, 5);

    left_motor_command_sub = nh.subscribe(left_motor_sub_topic, 1, &EarthRoverMicroControllerBridge::left_motor_command_callback, this);
    right_motor_command_sub = nh.subscribe(right_motor_sub_topic, 1, &EarthRoverMicroControllerBridge::right_motor_command_callback, this);

    ROS_INFO("Earth Rover Arduino bridge init done");
}

bool EarthRoverMicroControllerBridge::waitForPacket(const string packet)
{
    ros::Time begin = ros::Time::now();
    ros::Duration timeout = ros::Duration(5.0);

    while ((ros::Time::now() - begin) < timeout)
    {
        if (serial_ref.available()) {
            serial_buffer = serial_ref.readline();
            ROS_DEBUG("buffer: %s", serial_buffer.c_str());

            if (serial_buffer.compare(packet) == 0) {
                ROS_INFO("Earth Rover Arduino sent %s!", packet.c_str());
                return true;
            }
        }
    }

    ROS_ERROR("Timeout reached. Serial buffer didn't contain '%s', buffer: %s", packet.c_str(), serial_buffer.c_str());
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

    if (!waitForPacket(HELLO_MESSAGE)) {
        return 1;
    }
    if (!waitForPacket(READY_MESSAGE)) {
        return 1;
    }

    serial_ref.write(START_COMMAND);

    ros::Rate clock_rate(60);  // 60 Hz

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
    serial_buffer = serial_buffer.substr(ENCODER_MESSAGE_HEADER.size(), serial_buffer.size() - 1);
    ROS_DEBUG("buffer: %s", serial_buffer.c_str());

    size_t time_index = serial_buffer.find(MESSAGE_DELIMITER);
    ROS_DEBUG("encoder arduino time: %s", serial_buffer.substr(1, time_index).c_str());

    switch (serial_buffer.at(0)) {
        case 'r':
            right_encoder_msg.data = STR_TO_INT(serial_buffer.substr(time_index + 1));
            ROS_DEBUG("right_encoder_msg: %li", right_encoder_msg.data);

            right_encoder_pub.publish(right_encoder_msg);
            break;
        case 'l':
            left_encoder_msg.data = STR_TO_INT(serial_buffer.substr(time_index + 1));
            ROS_DEBUG("left_encoder_msg: %li", left_encoder_msg.data);

            left_encoder_pub.publish(left_encoder_msg);
            break;
        default:
            ROS_WARN("Invalid segment type! Segment: '%c', packet: '%s'", serial_buffer.at(0), serial_buffer.c_str());
            break;
    }
}

void EarthRoverMicroControllerBridge::left_motor_command_callback(const std_msgs::Int8& motor_command)
{
    char buffer[MOTOR_COMMAND_MESSAGE_LEN];
    sprintf(buffer, "ml%04d\n", motor_command.data);
    ROS_DEBUG("writing %s", buffer);
    serial_ref.write(buffer);
}

void EarthRoverMicroControllerBridge::right_motor_command_callback(const std_msgs::Int8& motor_command)
{
    char buffer[MOTOR_COMMAND_MESSAGE_LEN];
    sprintf(buffer, "mr%04d\n", motor_command.data);
    ROS_DEBUG("writing %s", buffer);
    serial_ref.write(buffer);
}
