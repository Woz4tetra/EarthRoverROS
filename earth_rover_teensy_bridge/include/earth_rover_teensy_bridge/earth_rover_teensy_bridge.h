#ifndef _EARTH_ROVER_TEENSY_BRIDGE_H_
#define _EARTH_ROVER_TEENSY_BRIDGE_H_

#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include "serial/serial.h"

#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <sstream>
#include "boost/format.hpp"

#include "earth_rover_teensy_bridge/ActivationDistances.h"
#include "earth_rover_teensy_bridge/SensorStateMachine.h"

using namespace std;
using namespace earth_rover_teensy_bridge;

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

    XmlRpc::XmlRpcValue xml_parsed_activation_distances;

    string guard_lock_topic;
    string guard_topic;
    ros::Publisher guard_lock_pub;
    ros::Publisher guard_pub;

    string cmd_vel_topic_name;
    ros::Subscriber cmd_vel_sub;

    std_msgs::Bool guard_lock_msg;
    geometry_msgs::Twist guard_msg;

    string act_dist_service_name;
    ros::ServiceServer act_dist_service;

    double cmd_angular_speed, cmd_linear_velocity;
    double min_angular_activation_speed, min_linear_activation_speed;
    bool is_moving;

    // Wait for the packet header specified with a timeout
    bool waitForPacket(const string ask_packet, const string response_packet);

    void parseActDistToken(string token);
    void parseActDistMessage();

    void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg);

    void writeActivationDists();
    bool setActivationDists(ActivationDistances::Request &req, ActivationDistances::Response &res);

    void checkGuardState();

    int activated_sensor;
    double activated_dist;
    vector<SensorStateMachine>* sensor_states;

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

    static const string ACTIVATE_MESSAGE_HEADER;

    int run();
};

#endif  // _EARTH_ROVER_TEENSY_BRIDGE_H_
