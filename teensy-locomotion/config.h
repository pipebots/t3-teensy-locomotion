/*
*  Robot & Motor Configuration
*/
#ifndef config_h
#define config_h

#include "motor.h"

#define LED_PIN 13 //onboard LED
const unsigned int estop_pin = 23 ; // For monitoring e-stop button

const unsigned int deadman_timeout = 500; // ms
const unsigned int diagnostic_frequency = 1; // publishing frequency (Hz)

// Robot base parameters
const float max_speed = 1.0; // m/s
const float wheel_base = 0.08; // m

// Motor 1 Darth Sprintious
const char *driver_1_name = "Left Motor";
const char *driver_1_id = "Left MD13S";
const driver_type driver_1 = pwm_dir;
const unsigned int d1_pin_speed = 10;
const unsigned int d1_pin_dir = 12;
const unsigned int d1_deadzone = 5;

// Motor 2 Darth Sprintious
const char *driver_2_name = "Right Motor";
const char *driver_2_id = "Right MD13S";// Cytron MD13S"; limit of 11 chars?!?!
const driver_type driver_2 = pwm_dir;
const unsigned int d2_pin_speed = 9;
const unsigned int d2_pin_dir = 11;
const unsigned int d2_deadzone = 5;

// Encoder 1
const char *encoder_1_name = "Left Encoder";
const char *encoder_1_id = "";
const unsigned int en1_pin_a = 3;
const unsigned int en1_pin_b = 4;
const unsigned int en1_counts_per_rev = 2500;
const bool en1_inverse = true;

// Encoder 2
const char *encoder_2_name = "Right Encoder";
const char *encoder_2_id = "";
const unsigned int en2_pin_a = 5;
const unsigned int en2_pin_b = 6;
const unsigned int en2_counts_per_rev = 2500;
const bool en2_inverse = false;


/*
// Motor 1 Rover5
const char *driver_1_name = "Left Motor";
const char *driver_1_id = "Left SN754410";
const driver_type driver_1 = h_bridge;
const unsigned int d1_pin_en = 5;
const unsigned int d1_pin_a = 7;
const unsigned int d1_pin_b = 8;
const unsigned int d1_deadzone = 5;

// Motor 2 Rover5
const char *driver_2_name = "Right Motor";
const char *driver_2_id = "Right SN754410";
const driver_type driver_2 = h_bridge;
const unsigned int d2_pin_en = 23;
const unsigned int d2_pin_a = 17;
const unsigned int d2_pin_b = 16;
const unsigned int d2_deadzone =5;
*/
#endif
