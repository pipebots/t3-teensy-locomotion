# Teensy Bot
A test of [micro-ros for arduino](https://github.com/micro_ros_ardino) on a Teensy 3.2 board.

This is a simple driver for a differential drive robot base.

### Features
- Uses standard interfaces - accepts twist messages on /cmd_vel topic.
- Has deadmand timer so the motors are set to 0 if no command is received for a set period.
- Error signalling via onboard LED.

# Install
Follow the [instructions](https://github.com/micro-ROS/micro_ros_arduino/blob/foxy/README.md) to install micro-ros.
You may find [my notes](https://n-fry.gitbook.io/ros2-notes/package-tests/micro-ros-for-arduino-ide) useful too.

Then clone this repo and upload the `.ino` file to the Teensy via the Arduino IDE.

# Config
The `config.h` file is used to set the parameters of the robot, edit this for your hardware.
Currently supports two types of motor driver:
- H Bridge (SN754410 in my case)
	- `driver_type = h_bridge`
	- Three pin numbers need setting; Enable (used for speed control) and input 1 & 2 of the bridge (for direction).
- Direction and Speed style motor drivers
	- `driver_type = pwm_dir`
	- Two pin numbers need setting; PWM(or speed) and Direction pins.

# Usage
You need to send Twist messages to command the robot, I used a joystick and [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy/tree/foxy) for this, but you could just use a keyboard package instead. See [these notes](https://n-fry.gitbook.io/ros2-notes/package-tests/teleop) for ROS2 teleop options.

# Subscribed Topics
- `cmd_vel (geometry_msgs/msg/Twist)`
	- Command velocity messages. Only the Linear X and Angular Z velocities are used.

# Error Codes
The onboard LED is used to signal the state of the board.
````
LED         |	Status              |
------------|---------------------|
Off         | Ok                  |
On          | Message Timeout     |
Flash [1:1]	| Communication Error |
Flash [5:1]	| Configuration Error |
````
## Message/Deadman Timeout
As a safety feature there is an internal timer which sets the motors to stationary if no message is received on the `cmd_vel` topic for 500ms.

## Communication Error
If the board looses it's connection to the micro-ros agent then it will enter this state. The LED will blink rapidly with equal on/off times for 5 seconds. The board will then reset in an attempt to reconnect.

## Configuration Error
If the code detects that the incorrect parameters have been set in the motor setup function for the type of driver declared in `config.h` it will enter and remain in this error state. The LED will flash with a long/short on/off cycle.
