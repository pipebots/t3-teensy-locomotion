#ifndef neopixel_h
#define neopixel_h

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>
#include <pipebot_msgs/msg/leds.h>

const char* ring_colour(int8_t, uint8_t, Adafruit_NeoPixel *);
void loading_chase_double(unsigned int, uint32_t, int, Adafruit_NeoPixel,
                            Adafruit_NeoPixel);
void loading_chase(unsigned int, uint32_t, int, Adafruit_NeoPixel);
#endif
