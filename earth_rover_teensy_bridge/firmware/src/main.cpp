#include <Arduino.h>
#include <HC_SR04_x6.h>

// minimum reliable distance: 5cm

#define ECHO_PIN_0 8
#define ECHO_PIN_1 7
#define ECHO_PIN_2 6
#define ECHO_PIN_3 5
#define ECHO_PIN_4 4
#define ECHO_PIN_5 3
#define TRIG_PIN   2

HC_SR04_x6 sensor_0(TRIG_PIN, ECHO_PIN_0, digitalPinToInterrupt(ECHO_PIN_0), 0);
HC_SR04_x6 sensor_1(TRIG_PIN, ECHO_PIN_1, digitalPinToInterrupt(ECHO_PIN_1), 1);
HC_SR04_x6 sensor_2(TRIG_PIN, ECHO_PIN_2, digitalPinToInterrupt(ECHO_PIN_2), 2);
HC_SR04_x6 sensor_3(TRIG_PIN, ECHO_PIN_3, digitalPinToInterrupt(ECHO_PIN_3), 3);
HC_SR04_x6 sensor_4(TRIG_PIN, ECHO_PIN_4, digitalPinToInterrupt(ECHO_PIN_4), 4);
HC_SR04_x6 sensor_5(TRIG_PIN, ECHO_PIN_5, digitalPinToInterrupt(ECHO_PIN_5), 5);

void setup()
{
    Serial.begin(115200);

    sensor_0.begin();
    sensor_1.begin();
    sensor_2.begin();
    sensor_3.begin();
    sensor_4.begin();
    sensor_5.begin();

    Serial.println("Setup complete");
}

void loop()
{
    // Serial.println("ping!");
    sensor_0.start();
    sensor_1.start();
    sensor_2.start();
    sensor_3.start();
    sensor_4.start();
    sensor_5.start();

    while (!sensor_0.isFinished() &&
           !sensor_1.isFinished() &&
           !sensor_2.isFinished() &&
           !sensor_3.isFinished() &&
           !sensor_4.isFinished() &&
           !sensor_5.isFinished()) {
       delayMicroseconds(500);
    }

    Serial.print(sensor_0.getRange()); Serial.print("cm, ");
    Serial.print(sensor_1.getRange()); Serial.print("cm, ");
    Serial.print(sensor_2.getRange()); Serial.print("cm, ");
    Serial.print(sensor_3.getRange()); Serial.print("cm, ");
    Serial.print(sensor_4.getRange()); Serial.print("cm, ");
    Serial.print(sensor_5.getRange()); Serial.println("cm");

    delay(50);
}

/*
#define TRIGGER_PIN 2
#define NUM_SENSORS 6
int echo_pins[NUM_SENSORS] = {8, 7, 6, 5, 4, 3};
volatile uint32_t echo_times[NUM_SENSORS];
volatile double distances_cm[6];

uint32_t trigger_time;

void echo_1_interrupt() {
    echo_times[0] = micros();
}

void trigger_sensors()
{
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(5);

    digitalWrite(TRIGGER_PIN, HIGH);
    trigger_time = micros();
    delayMicroseconds(10);

    digitalWrite(TRIGGER_PIN, LOW);
}

void calculate_distances()
{
    for (size_t i = 0; i < NUM_SENSORS; i++) {
        distances_cm[i] = (echo_times[i] - trigger_time / 2) / 29.1;
    }
}

void setup() {
    pinMode(TRIGGER_PIN, OUTPUT);
    for (size_t i = 0; i < NUM_SENSORS; i++) {
        pinMode(echo_pins[i], INPUT);
    }

    // attachInterrupt(digitalPinToInterrupt(echo_pins[0]), echo_1_interrupt, FALLING);
}

void loop() {
    trigger_sensors();
    // uint32_t duration = pulseIn(echo_pins[0], HIGH);
    // double cm = (duration / 2) / 29.1;
    // Serial.print(cm);

    Serial.print(distances_cm[0]);
    Serial.print("cm");
    Serial.println();
    delay(100);
}
*/
