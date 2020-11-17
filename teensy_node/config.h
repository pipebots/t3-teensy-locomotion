/*
*  Robot & Motor Configuration
*/
#include "motor.h"

#define LED_PIN 13 //onboard LED

const unsigned int deadman_timeout = 500; //ms

const float max_speed = 1.0; // m/s
const float wheel_base = 0.08; // m

const driver_type left_driver = h_bridge;
const unsigned int left_pin_en = 5;
const unsigned int left_pin_a = 7;
const unsigned int left_pin_b = 8;
const unsigned int left_deadzone = 5;

const driver_type right_driver = h_bridge;
const unsigned int right_pin_en = 23;
const unsigned int right_pin_a = 17;
const unsigned int right_pin_b = 16;
const unsigned int right_deadzone =5;
