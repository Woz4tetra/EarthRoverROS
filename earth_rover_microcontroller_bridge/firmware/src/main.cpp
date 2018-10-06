#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <SerialManager.h>
#include <Adafruit_MotorShield.h>

#define ENCODER_PRINT_THRESHOLD 50

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *right_motor = AFMS.getMotor(1);
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);

int left_speed = 0;
int right_speed = 0;

Encoder left_enc(3, 4);
Encoder right_enc(5, 2);

long left_enc_pos = -1;
long right_enc_pos = -1;

uint32_t prev_print_time = 0;

SerialManager manager;

void setup() {
    // manager.begin();  /// this doesn't work for some reason...
    Serial.begin(115200);
    manager.writeHello();

    AFMS.begin();

    manager.writeReady();
}

void set_left_speed(int speed)
{
    if (speed < 0) {
        left_motor->run(BACKWARD);
        left_motor->setSpeed(abs(speed));
    }
    else if (speed > 0) {
        left_motor->run(FORWARD);
        left_motor->setSpeed(speed);
    }
    else {
        left_motor->run(BRAKE);
        left_motor->setSpeed(speed);
    }
}

void set_right_speed(int speed)
{
    if (speed < 0) {
        right_motor->run(BACKWARD);
        right_motor->setSpeed(abs(speed));
    }
    else if (speed > 0) {
        right_motor->run(FORWARD);
        right_motor->setSpeed(speed);
    }
    else {
        right_motor->run(BRAKE);
        right_motor->setSpeed(speed);
    }
}

void loop() {
    if (manager.available()) {
        int status = manager.readSerial();
        String command = manager.getCommand();

        if (status == 2)  // start event
        {
            left_motor->run(BRAKE);
            right_motor->run(BRAKE);
            left_enc_pos = 0;
            right_enc_pos = 0;

            left_enc.write(0);
            right_enc.write(0);

            prev_print_time = millis();
        }
        else if (status == 1)  // stop event
        {
            left_motor->run(RELEASE);
            right_motor->run(RELEASE);
        }
        else if (status == 0)  // user command
        {
            if (command.charAt(0) == 'm') {
                if (command.charAt(1) == 'l') {
                    left_speed = -command.substring(2).toInt();
                    set_left_speed(left_speed);

                }
                else if (command.charAt(1) == 'r') {
                    right_speed = -command.substring(2).toInt();
                    set_right_speed(right_speed);
                }
            }
        }
    }

    if (!manager.isPaused()) {
        // long new_left_enc_pos = left_enc.read();
        // long new_right_enc_pos = right_enc.read();
        left_enc_pos = left_enc.read();
        right_enc_pos = right_enc.read();

        // if (abs(new_left_enc_pos - left_enc_pos) > ENCODER_PRINT_THRESHOLD ||
        //     abs(new_right_enc_pos - right_enc_pos) > ENCODER_PRINT_THRESHOLD) {
        if ((millis() - prev_print_time) > 250) {
            // left_enc_pos = new_left_enc_pos;
            // right_enc_pos = new_right_enc_pos;

            Serial.print("enc\tt");
            Serial.print(millis());
            Serial.print("\tl");
            Serial.print(left_enc_pos);
            Serial.print("\tr");
            Serial.print(right_enc_pos);
            Serial.print("\n");

            prev_print_time = millis();
        }
    }
}
