/*
*  Robot & Motor Configuration
*/
#ifndef config_h
#define config_h

#include "motor.h"

#define LED_PIN 13 //onboard LED

const unsigned int deadman_timeout = 500; // ms
const unsigned int diagnostic_frequency = 1; // Hz

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

const char *l_encoder_name = "Left Encoder";
const unsigned int l_encoder_pin_a = 1;
const unsigned int l_encoder_pin_b = 2;
const unsigned int l_counts_per_rev = 2500;
const char *l_encoder_id = "";
const bool l_inverse = false;

const char *r_encoder_name = "Right Encoder";
const unsigned int r_encoder_pin_a = 3;
const unsigned int r_encoder_pin_b = 4;
const unsigned int r_counts_per_rev = 2500;
const char *r_encoder_id = "";
const bool r_inverse = true;

#endif
