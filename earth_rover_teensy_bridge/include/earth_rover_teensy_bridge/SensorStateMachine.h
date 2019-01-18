#include "ros/ros.h"


enum ActivationStates { ACTIVATED, WAITING, STALE };


class SensorStateMachine {
private:
    double _activated_dist;
    double _activation_dist;
    double _activated_time;
    ros::Duration _activation_timeout;
    bool _state;

public:
    SensorStateMachine(double activation_dist, double activation_timeout);

    void update(int activated_sensor, double activated_dist);
    void set_to_waiting();
};
