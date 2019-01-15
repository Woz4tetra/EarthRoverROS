#include <Arduino.h>
#include <HC_SR04_x6.h>
#include <SerialManager.h>

// minimum reliable distance: 5cm

#define ECHO_PIN_1 8
#define ECHO_PIN_2 7
#define ECHO_PIN_3 6
#define ECHO_PIN_4 5
#define ECHO_PIN_5 4
#define ECHO_PIN_6 3
#define TRIG_PIN   2

#define NUM_SENSORS 6

// the "zeroth" sensor means no sensors are activated
HC_SR04_x6 sensor_1(TRIG_PIN, ECHO_PIN_1, digitalPinToInterrupt(ECHO_PIN_1), 0);
HC_SR04_x6 sensor_2(TRIG_PIN, ECHO_PIN_2, digitalPinToInterrupt(ECHO_PIN_2), 1);
HC_SR04_x6 sensor_3(TRIG_PIN, ECHO_PIN_3, digitalPinToInterrupt(ECHO_PIN_3), 2);
HC_SR04_x6 sensor_4(TRIG_PIN, ECHO_PIN_4, digitalPinToInterrupt(ECHO_PIN_4), 3);
HC_SR04_x6 sensor_5(TRIG_PIN, ECHO_PIN_5, digitalPinToInterrupt(ECHO_PIN_5), 4);
HC_SR04_x6 sensor_6(TRIG_PIN, ECHO_PIN_6, digitalPinToInterrupt(ECHO_PIN_6), 5);

uint32_t prev_ping_time = 0;
size_t num_act_dists_received = 0;

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
}

void loop()
{
    if (!manager.isPaused())
    {
        if (prev_ping_time > millis()) {
            prev_ping_time = millis();
        }
        // don't proceed until activation distances have been sent
        if (num_act_dists_received >= NUM_SENSORS && millis() - prev_ping_time > 50) {
            // Serial.println("ping!");
            sensor_1.start();
            sensor_2.start();
            sensor_3.start();
            sensor_4.start();
            sensor_5.start();
            sensor_6.start();

            while (!sensor_1.isFinished() &&
                   !sensor_2.isFinished() &&
                   !sensor_3.isFinished() &&
                   !sensor_4.isFinished() &&
                   !sensor_5.isFinished() &&
                   !sensor_6.isFinished()) {
               delayMicroseconds(100);
            }

            Serial.print("a\tt");
            Serial.print(millis());
            // only one sensor needs to activate at a time to tell ROS to cut the motor power
            if      (sensor_1.isActivated()) { Serial.print("\tn1\td"); Serial.print(sensor_1.getRange()); }
            else if (sensor_2.isActivated()) { Serial.print("\tn2\td"); Serial.print(sensor_2.getRange()); }
            else if (sensor_3.isActivated()) { Serial.print("\tn3\td"); Serial.print(sensor_3.getRange()); }
            else if (sensor_4.isActivated()) { Serial.print("\tn4\td"); Serial.print(sensor_4.getRange()); }
            else if (sensor_5.isActivated()) { Serial.print("\tn5\td"); Serial.print(sensor_5.getRange()); }
            else if (sensor_6.isActivated()) { Serial.print("\tn6\td"); Serial.print(sensor_6.getRange()); }
            else { Serial.print("n0\td0.0"); }
            Serial.print("\n");

            prev_ping_time = millis();
        }
    }

    if (manager.available()) {
        int status = manager.readSerial();
        String command = manager.getCommand();

        if (status == 2)  // start event
        {
            prev_ping_time = millis();
        }
        // else if (status == 1)  // stop event
        // {
        //
        // }
        else if (status == 0)  // user command
        {
            if (command.charAt(0) == 'd') {
                if (num_act_dists_received < NUM_SENSORS) {
                    num_act_dists_received++;
                }

                int index = command.substring(1, 2).toInt();
                float activation_dist = command.substring(4).toFloat();
                switch (index) {
                    case 1: sensor_1.setActDist(activation_dist); break;
                    case 2: sensor_2.setActDist(activation_dist); break;
                    case 3: sensor_3.setActDist(activation_dist); break;
                    case 4: sensor_4.setActDist(activation_dist); break;
                    case 5: sensor_5.setActDist(activation_dist); break;
                    case 6: sensor_6.setActDist(activation_dist); break;
                }
            }
        }
    }
}
