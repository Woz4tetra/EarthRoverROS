#include "earth_rover_teensy_bridge/SensorStateMachine.h"

SensorStateMachine::SensorStateMachine(double activation_dist, double activation_timeout)
{
    _activated_dist = -1.0;
    _activated_time = ros::Time::now();
    _state = false;

    _activation_timeout = activation_timeout;
    _activation_dist = activation_dist;
}

bool SensorStateMachine::update(double current_dist)
{
    // assuming a sensor activated

    ros::Time now = ros::Time::now();
    if (now - _activated_time < _activation_timeout) {
        ROS_INFO("ACTIVATED. waiting for timeout");
        return true;
    }

    if (_activated_dist < 0.0 || current_dist < _activated_dist) {
        ROS_INFO("ACTIVATED. prev dist: %f, act dist: %f", _activated_dist, current_dist);

        _state = true;
        _activation_time = now;
        _activated_dist = current_dist - 0.5;  // add a small buffer to account for sensor noise
        if (_activated_dist < 0.0) {
            _activated_dist = 0.0;
        }
    }
    else if (current_dist >= _activated_dist) {
        _state = false;
        ROS_INFO("STALE");
    }

    return _state;
}

void SensorStateMachine::set_to_waiting()
{
    _state = false;
    _activated_dist = -1.0;
    ROS_INFO("WAITING");
}
