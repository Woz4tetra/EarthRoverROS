#ifndef _EARTH_ROVER_TEENSY_BRIDGE_H_
#define _EARTH_ROVER_TEENSY_BRIDGE_H_

#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include "serial/serial.h"

#include <iostream>
#include <sstream>
#include "boost/format.hpp"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"

using namespace std;

// string parsing macros
#define STR_TO_FLOAT(string)  strtof((string).c_str(), 0)
#define STR_TO_INT(string) string_to_int64(string)

class EarthRoverTeensyBridge {
private:
    ros::NodeHandle nh; // ROS node handle
    string serial_port;
    int serial_baud;
    string serial_buffer;
    serial::Serial serial_ref;

    string ultrasonic_topic_name;
    ros::Publisher ultrasonic_pub;
    std_msgs::Float32MultiArray ultrasonic_msg;

    void setVectorLength(size_t length);
    void assignDist(size_t index, double dist);

    // Wait for the packet header specified with a timeout
    bool waitForPacket(const string ask_packet, const string response_packet);

    void parseActDistToken(string token);
    void parseActDistMessage();

public:
    EarthRoverTeensyBridge(ros::NodeHandle* nodehandle);

    // Important constants for the node's initialization
    static const string NODE_NAME;
    static const string PACKET_END;  // character that every packet ends with
    static const string HELLO_MESSAGE;  // the message to expect when the microcontroller starts up
    static const string READY_MESSAGE;  // message signalling that the microcontroller is ready to receive commands
    static const string READY_ASK_COMMAND;
    static const string START_COMMAND;  // packet to send to the microcontroller to tell it to start
    static const string STOP_COMMAND;  // packet to send to the microcontroller to tell it to stop
    static const string MESSAGE_DELIMITER;

    static const string ULTRASONIC_MESSAGE_HEADER;

    int run();
};

#endif  // _EARTH_ROVER_TEENSY_BRIDGE_H_
