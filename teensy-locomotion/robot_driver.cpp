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
#include "robot_driver.h"

/**
* @brief Differential drive robot base.
* @param max_speed Maximum speed the robot can travel in a straight line in m/s.
* @param wheel_base Distance between the two wheels, sometimes call the track. In m.
*/
RobotDriver::RobotDriver(float max_speed, float wheel_base) {
  max_speed_ = max_speed;
  wheel_base_ = wheel_base;
}

/**
* @brief Convert twist velocities to wheel speed for diff drive base.
* @param linear Linear velociy, in X driection, in m/s.
* @param angular Angular velociy, in Z axis, in rad/s.
*/
void RobotDriver::wheel_speeds(float linear, float angular) {
  left_speed = linear - angular*wheel_base_/2;
  right_speed = linear + angular*wheel_base_/2;
}

/*
* @brief Convert desired velocity to percentage of robot max speed.
* @note for use without encoders. When there is speed feedback use acutal speed not percentage.
* @param vel velocity in m/s.
* @return percentage speed, 0-100.
*/

int RobotDriver::percent_speed(float vel) {
  int percent = (100 * vel/max_speed_);
  return percent;
}
