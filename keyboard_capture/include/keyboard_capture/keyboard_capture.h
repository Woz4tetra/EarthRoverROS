#ifndef _KEYBOARD_CAPTURE_H_
#define _KEYBOARD_CAPTURE_H_

#include "ros/ros.h"

#include <iostream>
#include <sstream>

using namespace std;
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class KeyboardCapture
{
public:
    KeyboardCapture(ros::NodeHandle* nodehandle);
    void keyLoop();
private:
    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    ros::Publisher twist_pub_;
};


int kfd = 0;
struct termios cooked, raw;


void quit(int sig)
{
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

#endif  // _KEYBOARD_CAPTURE_H_
