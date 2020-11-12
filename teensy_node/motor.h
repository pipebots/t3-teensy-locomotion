/* Class to contain the motor and motor driver conf.
* inc. functions to drive motor
*/
#ifndef motor_h
#define motor_h

#include "Arduino.h"

// Pins currently for half SN754410 H-Bridge
class Motor{
    int pin_en_, pin_A_, pin_B_, deadzone_;
  public:
    void setup(int, int, int, int);
    void move_percent(int);
};

#endif
