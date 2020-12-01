#include "Arduino.h"
#include "motor.h"
#include "errors.h"

/*
* @brief Create motor and config the type of driver it is connected to
* @param driver Either a h_bridge or pwm_dir style driver.
*/
Motor::Motor(driver_type driver) {
  driver_type_ = driver;
}

/**
* @brief Set pin numbers and pin modes for SN754410 h-bridge
* @param pin_speed Enable pin for H-Bridge
* @param pin_1 H-Bridge pin 1
* @param pin_2 H-Bridge pin 2
* @param dead_zone Threshold below which the motor command signal will be pulled to zero.
* @return true if sucessful
*/
bool Motor::setup(int pin_speed, int pin_1, int pin_2, int dead_zone) {
  if (driver_type_ == h_bridge) {
    pin_en_ = pin_speed;
    pin_A_ = pin_1;
    pin_B_ = pin_2;
    deadzone_ = dead_zone;

    // set digital i/o pins as outputs:
    pinMode(pin_en_, OUTPUT);
    pinMode(pin_A_, OUTPUT);
    pinMode(pin_B_, OUTPUT);
    return true;
  } else {
    return false;
  }
}

/**
* @brief Set pin numbers and pin modes for motor drivers with PWM and Direction pins.
* @param pin_speed PWM or Speed pin of motor driver
* @param pin_dir Direction pin of motor driver
* @param dead_zone Threshold below which the motor command signal will be pulled to zero.
* @return true if sucessful
*/
bool Motor::setup(int pin_speed, int pin_dir, int dead_zone) {
  if (driver_type_ == pwm_dir) {
    pin_en_ = pin_speed;
    pin_A_ = pin_dir;
    deadzone_ = dead_zone;

    // set digital i/o pins as outputs:
    pinMode(pin_en_, OUTPUT);
    pinMode(pin_A_, OUTPUT);

    return true;
  } else {
    return false;
  }
}

/**
* @brief Convert percentage speed to pwm and send to motors
* @param percent_speed Speed command for the motor, 0-100%.
*/
void Motor::move_percent(int percent_speed) {
  int abs_speed = abs(percent_speed);
  abs_speed = constrain(abs_speed, 0, 100);  // check and constrain to 0-100%

  // add deadzone to turn off motors if PID is close to 0.
  if (abs_speed < deadzone_) {
    abs_speed = 0;
  }
  // conver % to pwm output
  int pwm = map(abs_speed, 0, 100, 0, 255);

  // set direction
  if (percent_speed >= 0) {
    move_fwd(pwm);
  } else {
    move_rev(pwm);
  }
}

/**
* @brief Make the motor turn forwards
* @param pwm Motor speed as PWM signal, 0-255.
*/
void Motor::move_fwd(int pwm) {
  switch (driver_type_) {
    case h_bridge:
      digitalWrite(pin_A_, HIGH);
      digitalWrite(pin_B_, LOW);
      break;
    case pwm_dir:
      digitalWrite(pin_A_, HIGH);
      break;
  }
  // set speed
  analogWrite(pin_en_, pwm);
}

/**
* @brief Make the motor turn backwards (reverse)
* @param pwm Motor speed as PWM signal, 0-255.
*/
void Motor::move_rev(int pwm) {
  switch (driver_type_) {
    case h_bridge:
      digitalWrite(pin_A_, LOW);
      digitalWrite(pin_B_, HIGH);
      break;
    case pwm_dir:
      digitalWrite(pin_A_, LOW);
      break;
  }
  // set speed
  analogWrite(pin_en_, pwm);
}
