#ifndef _EARTH_ROVER_MICROCONTROLLER_BRIDGE_H_
#define _EARTH_ROVER_MICROCONTROLLER_BRIDGE_H_

#include "ros/ros.h"
#include "serial/serial.h"

#include "std_msgs/Int8.h"
#include "std_msgs/Int64.h"

#include <iostream>
#include <sstream>

using namespace std;

// string parsing macros
#define STR_TO_FLOAT(string)  strtof((string).c_str(), 0)
#define STR_TO_INT(string) string_to_int64(string)

class EarthRoverMicroControllerBridge {
private:
    ros::NodeHandle nh; // ROS node handle
    string serial_port;
    int serial_baud;
    string serial_buffer;
    serial::Serial serial_ref;

    string left_enc_pub_topic;
    string right_enc_pub_topic;
    string left_motor_sub_topic;
    string right_motor_sub_topic;

    ros::Publisher left_encoder_pub;
    ros::Publisher right_encoder_pub;

    ros::Subscriber left_motor_command_sub;
    ros::Subscriber right_motor_command_sub;

    // Wait for the packet header specified with a timeout
    bool waitForPacket(const string packet);

    std_msgs::Int64 right_encoder_msg;
    std_msgs::Int64 left_encoder_msg;
    void parseToken(string token);
    void parseEncoderMessage();

    void left_motor_command_callback(const std_msgs::Int8& motor_command);
    void right_motor_command_callback(const std_msgs::Int8& motor_command);

public:
    EarthRoverMicroControllerBridge(ros::NodeHandle* nodehandle);

    // Important constants for the node's initialization
    static const string NODE_NAME;
    static const string PACKET_END;  // character that every packet ends with
    static const string HELLO_MESSAGE;  // the message to expect when the microcontroller starts up
    static const string READY_MESSAGE;  // message signalling that the microcontroller is ready to receive commands
    static const string START_COMMAND;  // packet to send to the microcontroller to tell it to start
    static const string STOP_COMMAND;  // packet to send to the microcontroller to tell it to stop
    static const string MESSAGE_DELIMITER;

    static const string ENCODER_MESSAGE_HEADER;

    static const size_t MOTOR_COMMAND_MESSAGE_LEN;

    int run();
};

#endif  // _EARTH_ROVER_MICROCONTROLLER_BRIDGE_H_
