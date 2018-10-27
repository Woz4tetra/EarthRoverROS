#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <SerialManager.h>

#define ENCODER_PRINT_THRESHOLD 50

Encoder encoder(2, 3);
long encoder_pos = 0;
uint32_t prev_print_time = 0;

SerialManager manager;

void setup() {
    // manager.begin();  /// this doesn't work for some reason...
    Serial.begin(115200);
    manager.writeHello();

    manager.writeReady();
}

void loop() {
    if (manager.available()) {
        int status = manager.readSerial();
        String command = manager.getCommand();

        if (status == 2)  // start event
        {
            encoder.write(0);
            prev_print_time = millis();
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

    if (!manager.isPaused()) {
        encoder_pos = encoder.read();

        // if ((millis() - prev_print_time) > 250) {

            prev_print_time = millis();
            Serial.print("enc\tt");
            Serial.print(prev_print_time);
            Serial.print("\tp");
            Serial.print(encoder_pos);
            Serial.print("\n");

            delay(10);
        // }
    }
}
