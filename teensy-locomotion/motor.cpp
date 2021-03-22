/*
* MIT License
*
* Copyright (c) 2021 University of Leeds
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#include "Arduino.h"
#include "motor.h"
/*
* @brief Create motor and config the type of driver it is connected to.
* @param motor_name Name for use in diagnostic messages.
* @param id Hardware Id or serial number for use in diagnostic messages.
* @param driver Either a h_bridge or pwm_dir style driver.
*/
Motor::Motor(const char *motor_name, const char *id, driver_type driver) {
  name = motor_name;
  hardware_id = id;
  driver_type_ = driver;
}

/**
* @brief Set pin numbers and pin modes for SN754410 h-bridge
* @param pin_speed Enable pin for H-Bridge
* @param pin_1 H-Bridge pin 1
* @param pin_2 H-Bridge pin 2
* @param dead_zone Threshold below which the motor command signal will be pulled to zero.
* @param inverse Swap the direction of rotation.
* @return true if sucessful
*/
bool Motor::setup(int pin_speed, int pin_1, int pin_2, int dead_zone, bool inverse) {
  if (driver_type_ == h_bridge) {
    pin_en_ = pin_speed;
    pin_A_ = pin_1;
    pin_B_ = pin_2;
    deadzone_ = dead_zone;
    inverse_ = inverse;

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
bool Motor::setup(int pin_speed, int pin_dir, int dead_zone, bool inverse) {
  if (driver_type_ == pwm_dir) {
    pin_en_ = pin_speed;
    pin_A_ = pin_dir;
    deadzone_ = dead_zone;
    inverse_ = inverse;

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
  if (inverse_) {
    switch (driver_type_) {
      case h_bridge:
        digitalWrite(pin_A_, LOW);
        digitalWrite(pin_B_, HIGH);
        break;
      case pwm_dir:
        digitalWrite(pin_A_, LOW);
        break;
    }
  } else {
      switch (driver_type_) {
        case h_bridge:
          digitalWrite(pin_A_, HIGH);
          digitalWrite(pin_B_, LOW);
          break;
        case pwm_dir:
          digitalWrite(pin_A_, HIGH);
          break;
      }
  }
  // set speed
  analogWrite(pin_en_, pwm);
}

/**
* @brief Make the motor turn backwards (reverse)
* @param pwm Motor speed as PWM signal, 0-255.
*/
void Motor::move_rev(int pwm) {
  if (inverse_) {
    switch (driver_type_) {
      case h_bridge:
        digitalWrite(pin_A_, HIGH);
        digitalWrite(pin_B_, LOW);
        break;
      case pwm_dir:
        digitalWrite(pin_A_, HIGH);
        break;
    }
  } else {
      switch (driver_type_) {
        case h_bridge:
          digitalWrite(pin_A_, LOW);
          digitalWrite(pin_B_, HIGH);
          break;
        case pwm_dir:
          digitalWrite(pin_A_, LOW);
          break;
      }
  }
  // set speed
  analogWrite(pin_en_, pwm);
}

/**
* @brief Return value of deadzone for the motor
* @return deadzone_
*/
int Motor::get_deadzone() {
  return deadzone_;
}
