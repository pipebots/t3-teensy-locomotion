/* Class to contain the motor and motor driver conf.
* inc. functions to drive motor
*/
#ifndef motor_h
#define motor_h

#include "Arduino.h"

enum driver_type{
  h_bridge,
  pwm_dir
};

/*
* Motor class can be configured to use either h-bridge
* or speed and direction style motor drivers.
*/
class Motor{
    int pin_en_, pin_A_, pin_B_, deadzone_;
    driver_type driver_type_;

  public:
    Motor(driver_type);
    void setup(int, int, int, int);
    void setup(int, int, int);
    void move_percent(int);
    void move_fwd(int);
    void move_rev(int);
};

#endif
