/* Class to contain the robot description and
*  functions for converting velocity to wheel pwm.
*/
#ifndef robot_driver_h
#define robot_driver_h

#include "Arduino.h"

// max_speed_ in m/s
// wheel_base_ in m
class RobotDriver{
    float max_speed_, wheel_base_;
  public:
    RobotDriver(float, float);
    void wheel_speeds(float, float);
    int percent_speed(float);
    float right_speed;
    float left_speed;
};

#endif
