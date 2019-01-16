#include "HC_SR04_x6.h"

//HC_SR04_x6 *HC_SR04_x6::_instance=NULL;
HC_SR04_x6 *HC_SR04_x6::_instance0(NULL);
HC_SR04_x6 *HC_SR04_x6::_instance1(NULL);
HC_SR04_x6 *HC_SR04_x6::_instance2(NULL);
HC_SR04_x6 *HC_SR04_x6::_instance3(NULL);
HC_SR04_x6 *HC_SR04_x6::_instance4(NULL);
HC_SR04_x6 *HC_SR04_x6::_instance5(NULL);

HC_SR04_x6::HC_SR04_x6(int trigger, int echo, int interrupt, int instance_index, float max_dist, bool use_cm, float activation_dist)
: _trigger(trigger), _echo(echo), _int(interrupt), _instance_index(instance_index), _activation_dist(activation_dist),
  _max(max_dist), _use_cm(use_cm), _finished(false)
{
    switch (instance_index) {
        case 0: if (_instance0 == 0) _instance0 = this; break;
        case 1: if (_instance1 == 0) _instance1 = this; break;
        case 2: if (_instance2 == 0) _instance2 = this; break;
        case 3: if (_instance3 == 0) _instance3 = this; break;
        case 4: if (_instance4 == 0) _instance4 = this; break;
        case 5: if (_instance5 == 0) _instance5 = this; break;
    }
}

void HC_SR04_x6::begin()
{
    pinMode(_trigger, OUTPUT);
    digitalWrite(_trigger, LOW);
    pinMode(_echo, INPUT);
    switch (_instance_index) {
        case 0: attachInterrupt(_int, _static_echo_isr_0, CHANGE); break;
        case 1: attachInterrupt(_int, _static_echo_isr_1, CHANGE); break;
        case 2: attachInterrupt(_int, _static_echo_isr_2, CHANGE); break;
        case 3: attachInterrupt(_int, _static_echo_isr_3, CHANGE); break;
        case 4: attachInterrupt(_int, _static_echo_isr_4, CHANGE); break;
        case 5: attachInterrupt(_int, _static_echo_isr_5, CHANGE); break;
    }
}

void HC_SR04_x6::start()
{
    _finished=false;
    digitalWrite(_trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigger, LOW);
}

float HC_SR04_x6::_durationToUnits(unsigned long duration) {
    return (float)(duration) / ((_use_cm) ? 58.0:148.0);
}

float HC_SR04_x6::getRange()
{
    float result = _durationToUnits(_end - _start);
    if (result > _max) {
        result = _max;
    }
    return result;
}

bool HC_SR04_x6::isActivated() {
    float range = getRange();
    return range != 0.0 && range <= _activation_dist;
}

void HC_SR04_x6::setActDist(float activation_dist) {
    _activation_dist = activation_dist;
}

bool HC_SR04_x6::isFinished()
{
    if (!_finished && micros() - _start > ECHO_TIMEOUT_US) {
        return true;
    }
    return _finished;
}

void HC_SR04_x6::echo_isr()
{
    switch(digitalRead(_echo)){
        case HIGH:
            _start=micros();
            break;
        case LOW:
            _end=micros();
            _finished=true;
            break;
    }
}

void HC_SR04_x6::_static_echo_isr_0() { HC_SR04_x6::instance(0)->echo_isr(); }
void HC_SR04_x6::_static_echo_isr_1() { HC_SR04_x6::instance(1)->echo_isr(); }
void HC_SR04_x6::_static_echo_isr_2() { HC_SR04_x6::instance(2)->echo_isr(); }
void HC_SR04_x6::_static_echo_isr_3() { HC_SR04_x6::instance(3)->echo_isr(); }
void HC_SR04_x6::_static_echo_isr_4() { HC_SR04_x6::instance(4)->echo_isr(); }
void HC_SR04_x6::_static_echo_isr_5() { HC_SR04_x6::instance(5)->echo_isr(); }
