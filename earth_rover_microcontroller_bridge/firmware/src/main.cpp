#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <SerialManager.h>
#include "NeopixelPattern.h"

#define LED_STRIP_PIN 6
#define NUM_LEDS 24
#define BRIGHTNESS 255
void on_pattern_complete();

NeoPatterns strip(NUM_LEDS, LED_STRIP_PIN, NEO_GRBW + NEO_KHZ800, &on_pattern_complete);

uint32_t cool_color = strip.Color(0, 0, 255);
uint32_t hot_color = strip.Color(255, 0, 0);

#define ENCODER_PRINT_THRESHOLD 50

Encoder encoder(2, 3);
long encoder_pos = 0;
uint32_t prev_print_time = 0;

SerialManager manager;


void pulseWhite(uint8_t wait, unsigned int increment) {
    if (increment > 50) {
        increment = 50;
    }
    for(int j = 0; j < 256 ; j += increment){
        for(uint16_t i=0; i<strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(0,0,0, strip.WhiteGamma(j)));
        }
        if (wait > 0) {
            delay(wait);
        }
        strip.show();
    }
    for(int j = 255; j >= 0 ; j -= increment){
        for(uint16_t i=0; i<strip.numPixels(); i++) {
            strip.setPixelColor(i, strip.Color(0,0,0, strip.WhiteGamma(j)));
        }
        if (wait > 0) {
            delay(wait);
        }
        strip.show();
    }
}



void on_pattern_complete()
{
    if (strip.ActivePattern == FADE) {
        if (strip.Color2 == strip.Color(0, 0, 0)) {
            strip.ActivePattern = NONE;  // stop the strip if fading to off
            strip.show();
        }
        else {
            strip.Reverse();
        }
    }
}

uint32_t char_to_color(char color_char) {
    switch (color_char) {
        case 'w': return strip.Color(0, 0, 0, 255);
        case 'b': return strip.Color(0, 0, 255);
        case 'r': return strip.Color(255, 0, 0);
        case 'g': return strip.Color(0, 255, 0);
        case 'y': return strip.Color(255, 255, 0);
        default: return strip.Color(0, 0, 0);
    }
}

void fade_to_off() {
    strip.Fade(strip.Color(0, 0, 0, 255), strip.Color(0, 0, 0, 0), 256, 5);
}

void set_to_white() {
    strip.ActivePattern = NONE;
    strip.ColorSet(strip.Color(255, 255, 255, 255));
}

void set_to_off() {
    strip.ActivePattern = NONE;
    strip.ColorSet(strip.Color(0, 0, 0, 0));
}

void set_rainbow_type(char rainbow_type)
{
    if (rainbow_type == 's') {
        strip.RainbowCycle(50);
    }
    else if (rainbow_type == 'f') {
        strip.RainbowCycle(5);
    }
}


void setup() {
    // manager.begin();  /// this doesn't work for some reason...
    Serial.begin(115200);
    manager.writeHello();

    #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
    #endif
    // End of trinket special code
    strip.setBrightness(BRIGHTNESS);
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    pulseWhite(1, 2);
    strip.show();

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
            // set_rainbow_type('s');
        }
        else if (status == 1)  // stop event
        {
            fade_to_off();
        }
        else if (status == 0)  // user command
        {
            Serial.println(command);
            switch(command.charAt(0)) {
        		case 'o': fade_to_off(); break;
                case '-': set_to_off(); break;
        		case 'w': set_to_white(); break;
        		case 'r': set_rainbow_type(command.charAt(1)); break;
        		case 'f': strip.Fade(char_to_color(command.charAt(1)), strip.Color(0, 0, 0, 1), 255, 5); break;
        		case 'c': strip.Scanner(char_to_color(command.charAt(1)), 50); break;
        	}
        }
    }

    strip.Update();

    if (!manager.isPaused()) {
        encoder_pos = encoder.read();

        if ((millis() - prev_print_time) > 10) {
            prev_print_time = millis();
            Serial.print("enc\tt");
            Serial.print(prev_print_time);
            Serial.print("\tp");
            Serial.print(encoder_pos);
            Serial.print("\n");
        }
    }
}
