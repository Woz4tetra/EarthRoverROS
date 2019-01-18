
#ifndef _SENSOR_STATE_MACHINE_H_
#define _SENSOR_STATE_MACHINE_H_

#include "ros/ros.h"

enum ActivationStates { ACTIVATED, WAITING, STALE };


class SensorStateMachine {
private:
    double _activated_dist;
    double _activation_dist;
    ros::Time _activated_time;
    ros::Duration _activation_timeout;
    bool _state;

public:
    SensorStateMachine(double activation_dist, double activation_timeout);

    bool update(double activated_dist);
    // void set_to_waiting();
    void set_act_dist(double activation_dist);
    double get_act_dist();
};

#endif  // _SENSOR_STATE_MACHINE_H_
