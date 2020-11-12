#include "Arduino.h"
#include "motor.h"

/**
* @brief Set pin numbers and pin modes
* @note Currently for SN754410 h-bridge
* @param enable Enable pin for H-Bridge
* @param pin_1 H-Bridge pin 1
* @param pin_2 H-Bridge pin 2
* @param dead_zone Threshold below which the motor command signal will be pulled to zero.
*/
void Motor::setup(int enable, int pin_1, int pin_2, int dead_zone){
  pin_en_ = enable;
  pin_A_ = pin_1;
  pin_B_ = pin_2;
  deadzone_ = dead_zone;

  // set digital i/o pins as outputs:
  pinMode(pin_en_, OUTPUT);
  pinMode(pin_A_, OUTPUT);
  pinMode(pin_B_, OUTPUT);
}

/**
* @brief Convert percentage speed to pwm and send to motors
* @param percent_speed Speed command for the motor, 0-100%.
*/
void Motor::move_percent(int percent_speed){
  int abs_speed = abs(percent_speed);
  abs_speed = constrain(abs_speed, 0, 100); //check and constrain to 0-100%

  //add deadzone to turn off motors if PID is close to 0.
  if (abs_speed < deadzone_){
    abs_speed = 0;
  }
  //conver % to pwm output
  int pwm = map(abs_speed, 0, 100, 0, 255);

  // set direction
  if(percent_speed >= 0){
    digitalWrite(pin_A_, HIGH);
    digitalWrite(pin_B_, LOW);
  }
  else{
    digitalWrite(pin_A_, LOW);
    digitalWrite(pin_B_, HIGH);

  }
  // set speed
  analogWrite(pin_en_, pwm);
}
