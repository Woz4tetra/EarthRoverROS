#ifndef HC_SR04_X6_H
#define HC_SR04_X6_H

#include <Arduino.h>

#define ECHO_TIMEOUT_US 30000

class HC_SR04_x6 {
public:
    HC_SR04_x6(int trigger, int echo, int interrupt, int instance_index, float max_dist=200.0, bool use_cm=true);

    void begin();
    void start();
    bool isFinished();
    float getRange();
    static HC_SR04_x6* instance(int instance_index)
    {
        switch (instance_index) {
            case 0: return _instance0; break;
            case 1: return _instance1; break;
            case 2: return _instance2; break;
            case 3: return _instance3; break;
            case 4: return _instance4; break;
            case 5: return _instance5; break;
            default: return NULL;
        }
    }

    void echo_isr();

private:
    static void _static_echo_isr_0();
    static void _static_echo_isr_1();
    static void _static_echo_isr_2();
    static void _static_echo_isr_3();
    static void _static_echo_isr_4();
    static void _static_echo_isr_5();

    float _durationToUnits(unsigned long duration);

    int _trigger, _echo, _int, _instance_index;
    float _max;
    bool _use_cm;
    volatile unsigned long _start, _end;
    volatile bool _finished;

    static HC_SR04_x6* _instance0;
    static HC_SR04_x6* _instance1;
    static HC_SR04_x6* _instance2;
    static HC_SR04_x6* _instance3;
    static HC_SR04_x6* _instance4;
    static HC_SR04_x6* _instance5;
};

#endif
