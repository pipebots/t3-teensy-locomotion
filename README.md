# Teensy Bot
A test of [micro-ros for arduino](https://github.com/micro_ros_ardino) on a Teensy 3.2 board.

This is a simple driver for a differential drive robot base.

### Features
- Uses standard interfaces - accepts twist messages on /cmd_vel topic.
- Has deadmand timer so the motors are set to 0 if no command is recieved for a set period.

## Install
Follow the [instructions](https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/README.md) to install micro-ros.
You may find [my notes](https://n-fry.gitbook.io/ros2-notes/package-tests/micro-ros-for-arduino-ide) useful too.

Then clone this repo and upload the `.ino` file to the Teensy via the Arduino IDE.

## Config
The `config.h` file is used to set the parameters of the robot, edit this for your hardware.
Currently the motor drivers pin config is for SN754410 H-Bridges.

## Usage
You need to send Twist messages to command the robot, I used a joystick and [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy/tree/foxy) for this, but you could just use a keyboard package instead. See [these notes](https://n-fry.gitbook.io/ros2-notes/package-tests/teleop) for ROS2 teleop options.

## Subscribed Topics
- `cmd_vel (geometry_msgs/msg/Twist)` 
	- Command velocity messages. Only the Linear X and Angular Z velocities are used.
