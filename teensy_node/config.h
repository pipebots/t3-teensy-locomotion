/*
*  Robot & Motor Configuration
*/
#include "motor.h"

#define LED_PIN 13

const unsigned int deadman_timeout = 500;

const float max_speed = 1.0;
const float wheel_base = 0.08;

const driver_type left_driver = h_bridge;
const unsigned int left_pin_en = 3;
const unsigned int left_pin_a = 6;
const unsigned int left_pin_b = 7;
const unsigned int left_deadzone = 5;

const driver_type right_driver = h_bridge;
const unsigned int right_pin_en = 4;
const unsigned int right_pin_a = 8;
const unsigned int right_pin_b = 9;
const unsigned int right_deadzone =5;
