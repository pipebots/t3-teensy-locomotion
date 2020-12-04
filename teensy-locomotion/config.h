/*
*  Robot & Motor Configuration
*/
#ifndef config_h
#define config_h

#include "motor.h"

#define LED_PIN 13 //onboard LED

const unsigned int deadman_timeout = 500; // ms
const unsigned int diagnostic_frequency = 1; // publishing frequency (Hz)

// Robot base parameters
const float max_speed = 1.0; // m/s
const float wheel_base = 0.08; // m

// Motor 1
const char *driver_1_name = "Left Motor";
const char *driver_1_id = "Left SN754410";
const driver_type driver_1 = h_bridge;
const unsigned int d1_pin_en = 5;
const unsigned int d1_pin_a = 7;
const unsigned int d1_pin_b = 8;
const unsigned int d1_deadzone = 5;

// Motor 2
const char *driver_2_name = "Right Motor";
const char *driver_2_id = "Right SN754410";
const driver_type driver_2 = h_bridge;
const unsigned int d2_pin_en = 23;
const unsigned int d2_pin_a = 17;
const unsigned int d2_pin_b = 16;
const unsigned int d2_deadzone =5;

// Encoder 1
const char *encoder_1_name = "Left Encoder";
const char *encoder_1_id = "";
const unsigned int en1_pin_a = 1;
const unsigned int en1_pin_b = 2;
const unsigned int en1_counts_per_rev = 2500;
const bool en1_inverse = false;

// Encoder 2
const char *encoder_2_name = "Right Encoder";
const char *encoder_2_id = "";
const unsigned int en2_pin_a = 3;
const unsigned int en2_pin_b = 4;
const unsigned int en2_counts_per_rev = 2500;
const bool en2_inverse = true;

#endif
