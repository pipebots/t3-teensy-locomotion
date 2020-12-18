# Teensy Bot
A test of [micro-ros for arduino](https://github.com/micro_ros_ardino) on a Teensy 3.2 board.

This is a driver for a differential drive robot base.

### Features
- Uses standard interfaces - accepts twist messages on /cmd_vel topic.
- Has deadmand timer so the motors are set to 0 if no command is received for a set period.
- Error signalling via onboard LED.
- Publishes standard diagnostic messages.
- Emergency stop monitoring.
- Encoder data published.
- Error check for motion without encoder updates.
- Control RBGW neopixel rings

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
- `leds` ([pipebot_msgs/msg/Leds](https://github.com/pipebots/pipebot-msgs/blob/main/msg/Leds.msg))
	- Custom message which can be used for many different LED setups. In this case it is controlling one 24 LED neopixel ring (well two, but they wired to one pin).  
		`led = SIDE_LIGHTS`. `flash_mode` is ignored. To turn off set `brightness = 0`.

# Published Topics
- `diagnostics (diagnostic_msgs/msg/DiagnosticArray)`
	- Published at 1Hz (edit in config).
	- Best effort publisher
	- Formed of an array of component statuses:

		- Teensy Robot Driver `(diagnostic_msgs/msg/DiagnosticStatus)`
			- Key Value Pairs `(diagnostic_msgs/msg/KeyValue)`:
			- Deadman Timer - Off/On
			- Emergency Stop - Off/On
			- Side Lights - Colour

		- Left Motor Driver `(diagnostic_msgs/msg/DiagnosticStatus)`
		- Right Motor Driver `(diagnostic_msgs/msg/DiagnosticStatus)`
		- Left Encoder `(diagnostic_msgs/msg/DiagnosticStatus)`
		- Right Encoder `(diagnostic_msgs/msg/DiagnosticStatus)`
		- Battery `(diagnostic_msgs/msg/DiagnosticStatus)`

- `encoder_left` ([pipebot_msgs/msg/Encoders](https://github.com/pipebots/pipebot-msgs/blob/main/msg/Encoders.msg))
- `encoder_right` ([pipebot_msgs/msg/Encoders](https://github.com/pipebots/pipebot-msgs/blob/main/msg/Encoders.msg))
	- Encoder topics are:
	- Published at 20Hz (as per [pipbots spec](https://github.com/pipebots/pipebot-msgs/blob/main/specifications/locomotion-specification.md))
	- Best effort publishers

# Error Codes
The onboard LED is used to signal the state of the board. These errors are also reported back via the diagnostics topic.

LED         |	Status              |
------------|---------------------|
Off         | Ok                  |
On          | Message Timeout     |
Flash [1:1] | Communication Error |
Flash [5:1] | Configuration Error |

## Message/Deadman Timeout
As a safety feature there is an internal timer which sets the motors to stationary if no message is received on the `cmd_vel` topic for 500ms (can change in config). This sets the Robot driver component to a warning state.

## Communication Error
If the board looses it's connection to the micro-ros agent then it will enter this state. The LED will blink rapidly with equal on/off times for 5 seconds. The board will then reset in an attempt to reconnect. This does not attempt to report the error via the diagnostics is with no coms this would not be received anyway.

## Configuration Error
If the code detects that the incorrect parameters have been set in the motor setup function for the type of driver declared in `config.h` it will enter and remain in this error state. The LED will flash with a long/short on/off cycle.

## Encoder Fails to update
When the motors are commanded to move the value of the encoders is checked. If they do not update the component is set to an error state in the diagnostics.

## E-Stop Activated
Monitoring of an e-stop button is implemented and sets the driver to Error if pressed. The status read "Motors deactivated". A Key value pair also reports its status. Physically this disconnects the power from the motor drivers in my robot.

## Side lights key value errors
In the main driver status there is a key value pair for the LEDs. This updates with the colour set or 'off' if brightness is 0.
- `Colour Error` means there is no matching case for the `colour` set in the message. The LEDs turn ~yellow (30, 20, 0, 0).
- `LED not implemented` means there is no matching case for the `led` set in the message. Other LEDs do not update.
