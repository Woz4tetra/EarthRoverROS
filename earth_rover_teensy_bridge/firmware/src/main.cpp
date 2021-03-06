#include <Arduino.h>
#include <HC_SR04_x6.h>
#include <SerialManager.h>

// minimum reliable distance: 5cm

#define TRIG_PIN   2
// #define ECHO_PIN_1 3
#define ECHO_PIN_1 9  // switched pins mistakenly thinking there was a hardware issue
#define ECHO_PIN_2 4
#define ECHO_PIN_3 5
#define ECHO_PIN_4 6
#define ECHO_PIN_5 7
#define ECHO_PIN_6 8

#define LED_PIN    13

#define NUM_SENSORS 6

// the "zeroth" sensor means no sensors are activated
HC_SR04_x6 sensor_1(TRIG_PIN, ECHO_PIN_1, digitalPinToInterrupt(ECHO_PIN_1), 0);
HC_SR04_x6 sensor_2(TRIG_PIN, ECHO_PIN_2, digitalPinToInterrupt(ECHO_PIN_2), 1);
HC_SR04_x6 sensor_3(TRIG_PIN, ECHO_PIN_3, digitalPinToInterrupt(ECHO_PIN_3), 2);
HC_SR04_x6 sensor_4(TRIG_PIN, ECHO_PIN_4, digitalPinToInterrupt(ECHO_PIN_4), 3);
HC_SR04_x6 sensor_5(TRIG_PIN, ECHO_PIN_5, digitalPinToInterrupt(ECHO_PIN_5), 4);
HC_SR04_x6 sensor_6(TRIG_PIN, ECHO_PIN_6, digitalPinToInterrupt(ECHO_PIN_6), 5);

uint32_t prev_ping_time = 0;
uint32_t now = 0;

SerialManager manager;

void setup()
{
    Serial.begin(115200);
    delay(250);
    manager.writeHello();

    sensor_1.begin();
    sensor_2.begin();
    sensor_3.begin();
    sensor_4.begin();
    sensor_5.begin();
    sensor_6.begin();

    manager.writeReady();

    prev_ping_time = millis();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
}

void loop()
{
    if (manager.available()) {
        int status = manager.readSerial();
        String command = manager.getCommand();

        if (status == 2)  // start event
        {
            prev_ping_time = millis();
            Serial.print("u\tt");
            Serial.print(millis());
            Serial.print("\tl6");
            Serial.print("\n");
        }
        // else if (status == 1)  // stop event
        // {
        //
        // }
        // else if (status == 0)  // user command
        // {
        //
        // }
    }

    if (manager.isPaused()) {
        return;
    }

    now = millis();
    if (prev_ping_time > now) {
        prev_ping_time = now;
    }
    // setting this to 10 will cause sensor 4 to activate randomly
    if (now - prev_ping_time <= 20) {
        return;
    }
    // Serial.println("ping!");
    sensor_1.reset();
    sensor_2.reset();
    sensor_3.reset();
    sensor_4.reset();
    sensor_5.reset();
    sensor_6.reset();

    sensor_1.ping();

    while (!sensor_1.isFinished() &&
           !sensor_2.isFinished() &&
           !sensor_3.isFinished() &&
           !sensor_4.isFinished() &&
           !sensor_5.isFinished() &&
           !sensor_6.isFinished()) {
       delayMicroseconds(100);
    }

    Serial.print("u\tt");
    Serial.print(millis());
    Serial.print("\td00"); Serial.print(sensor_1.getRange());
    Serial.print("\td01"); Serial.print(sensor_2.getRange());
    Serial.print("\td02"); Serial.print(sensor_3.getRange());
    Serial.print("\td03"); Serial.print(sensor_4.getRange());
    Serial.print("\td04"); Serial.print(sensor_5.getRange());
    Serial.print("\td05"); Serial.print(sensor_6.getRange());
    Serial.print("\n");

    prev_ping_time = millis();
}
