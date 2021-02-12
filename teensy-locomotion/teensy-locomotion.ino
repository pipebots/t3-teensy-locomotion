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
*                                    __
* _________.__             ___.     |__|  __
* \______   \__|_____   ____\_ |__   _||__/  |_  ______
* |     ___/  \____ \_/ __ \| __ \ /  _ \   __\/  ___/
* |    |   |  |  |_> >  ___/| \_\ (  O_O )  |  \___ \
* |____|   |__|   __/ \___  >___  /\____/|__| /____  >
*             |__|        \/    \/                 \/
*
*  Low level controller for a differential drive robot base with a teensy 3.2.
*  It subscrives to cmd_vel and converts the Twist message to left and right motor commands.
*  The deadman_timer stops the motors if no command is recieved for 500ms.
*
*/

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <geometry_msgs/msg/twist.h>
#include <pipebot_msgs/msg/encoders.h>
#include <pipebot_msgs/msg/leds.h>
#include <pipebot_msgs/msg/led_config.h>
#include <pipebot_msgs/msg/motor_config.h>
#include <pipebot_msgs/msg/encoder_config.h>
#include <pipebot_msgs/msg/micro_params.h>
#include <pipebot_msgs/srv/update_micro_config.h>

#include "robot_driver.h"
#include "motor.h"
#include "config.h"
#include "diagnostics.h"
#include "encoder.h"
#include "neopixel.h"

rcl_subscription_t cmd_subscriber;
rcl_subscription_t led_subscriber;
rcl_publisher_t diagnostics_publisher;
rcl_publisher_t enc_1_publisher;
rcl_publisher_t enc_2_publisher;
rcl_service_t params_service;

pipebot_msgs__msg__Encoders encoder_1_data;
pipebot_msgs__msg__Encoders encoder_2_data;
pipebot_msgs__msg__Leds led_data;
pipebot_msgs__srv__UpdateMicroConfig_Response response;
pipebot_msgs__srv__UpdateMicroConfig_Response request;
geometry_msgs__msg__Twist cmd_twist;
diagnostic_msgs__msg__DiagnosticStatus * teensy_status;
diagnostic_msgs__msg__DiagnosticStatus * motor_1_status;
diagnostic_msgs__msg__DiagnosticStatus * motor_2_status;
diagnostic_msgs__msg__DiagnosticStatus * encoder_1_status;
diagnostic_msgs__msg__DiagnosticStatus * encoder_2_status;
diagnostic_msgs__msg__DiagnosticStatus * battery_status;
diagnostic_msgs__msg__KeyValue * deadman_keyval;
diagnostic_msgs__msg__KeyValue * estop;
diagnostic_msgs__msg__KeyValue * side_lights;
diagnostic_msgs__msg__KeyValue__Sequence teensy_key_array;
diagnostic_msgs__msg__DiagnosticStatus__Sequence status_array;
diagnostic_msgs__msg__DiagnosticArray * dia_array;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t deadman_timer;
rcl_timer_t diagnostic_timer;
rcl_timer_t encoder_timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {coms_error(&motor_1, &motor_2, LED_PIN);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {coms_error(&motor_1, &motor_2, LED_PIN);}}

int last_tick_1, last_tick_2 = 0;
RobotDriver robot(max_speed_mps, wheel_base_m);
Motor motor_1(driver_1_name, driver_1_id, driver_1);
Motor motor_2(driver_2_name, driver_2_id, driver_2);
Encoder encoder_1(encoder_1_name, en1_pin_a, en1_pin_b,
                  en1_counts_per_rev, encoder_1_id, en1_inverse);
Encoder encoder_2(encoder_2_name, en2_pin_a, en2_pin_b,
                  en2_counts_per_rev, encoder_2_id, en2_inverse);
Adafruit_NeoPixel ring_side(neo_side_num, led_side_pin, NEO_GRBW + NEO_KHZ800);

void params_service_callback(const void * request, void * response) {
  pipebot_msgs__srv__UpdateMicroConfig_Request * req_in =
    (pipebot_msgs__srv__UpdateMicroConfig_Request *) request;
  pipebot_msgs__srv__UpdateMicroConfig_Response * res_in =
    (pipebot_msgs__srv__UpdateMicroConfig_Response *) response;

  // deal with message here
  // const char *msg = "Parameters Recieved";
  // int len = strlen(msg)+1;
  // snprintf(res_in->message.data, len, msg);
  res_in->success = true;
}

void publish_diagnostics() {
  // update key value array
  teensy_key_array.data[0] = *deadman_keyval;
  teensy_key_array.data[1] = *estop;
  teensy_key_array.data[2] = *side_lights;
  teensy_status->values = teensy_key_array;

  // update status array
  status_array.data[0] = *teensy_status;
  status_array.data[1] = *motor_1_status;
  status_array.data[2] = *motor_2_status;
  status_array.data[3] = *encoder_1_status;
  status_array.data[4] = *encoder_2_status;
  status_array.data[5] = *battery_status;
  dia_array->status = status_array;

  RCSOFTCHECK(rcl_publish(&diagnostics_publisher, dia_array, NULL));
}

void vel_received_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  digitalWrite(LED_PIN, LOW);
  // reset deadman_timer
  rcl_timer_reset(&deadman_timer);

  // extract vels from twist message
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  // calc desired wheel speed (m/s)
  robot.wheel_speeds(linear, angular);

  // get as percentage of max speed
  // (once there is velocity feedback we can do that instead)
  int l_percent_speed = robot.percent_speed(robot.left_speed);
  int r_percent_speed = robot.percent_speed(robot.right_speed);

    // move motors
  motor_1.move_percent(l_percent_speed);
  motor_2.move_percent(r_percent_speed);

  // check encoders
  if (l_percent_speed > motor_1.get_deadzone() &&
        last_tick_1 == encoder_1.ticks) {
    // Motor moved but encoder value unchanged
    encoder_1_status = update_diagnostic_status(
                          encoder_1_status,
                          "Failed to update",
                           diagnostic_msgs__msg__DiagnosticStatus__ERROR);
  } else if (l_percent_speed > motor_1.get_deadzone() &&
        last_tick_1 != encoder_1.ticks) {
    encoder_1_status = update_diagnostic_status(
                          encoder_1_status,
                          "Ok",
                           diagnostic_msgs__msg__DiagnosticStatus__OK);
  }
  if (r_percent_speed > motor_2.get_deadzone() &&
        last_tick_2 == encoder_2.ticks) {
    // Motor moved but encoder value unchanged
    encoder_2_status = update_diagnostic_status(
                          encoder_2_status,
                          "Failed to update",
                           diagnostic_msgs__msg__DiagnosticStatus__ERROR);
  } else if (r_percent_speed > motor_2.get_deadzone() &&
        last_tick_2 != encoder_2.ticks) {
    encoder_2_status = update_diagnostic_status(
                          encoder_2_status,
                          "Ok",
                           diagnostic_msgs__msg__DiagnosticStatus__OK);
  }
  last_tick_1 = encoder_1.ticks;
  last_tick_2 = encoder_2.ticks;

  deadman_keyval = update_diagnostic_KeyValue(deadman_keyval, "Off");
  teensy_status = update_diagnostic_status(
                    teensy_status,
                    "cmd_vel recieved",
                    diagnostic_msgs__msg__DiagnosticStatus__OK);
}
void led_received_callback(const void * msgin) {
  const pipebot_msgs__msg__Leds * msg = (const pipebot_msgs__msg__Leds *)msgin;
  Adafruit_NeoPixel strip;
  const char* state;

  if (strcmp(msg->led.data, "sides") == 0) {
      state = ring_colour(msg->colour, msg->brightness, &ring_side);
      side_lights = update_diagnostic_KeyValue(side_lights, state);
      if (msg->brightness == 0) {
        side_lights = update_diagnostic_KeyValue(side_lights, "Off");
      }
    } else {
      // no matching cases
      side_lights = update_diagnostic_KeyValue(side_lights, "LED not implemented");
    }
}

// If no commands are recieved this executes and sets motors to 0
void deadman_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    motor_1.move_percent(0);
    motor_2.move_percent(0);

    teensy_status = update_diagnostic_status(
                      teensy_status,
                      "No cmd_vel for 500ms",
                      diagnostic_msgs__msg__DiagnosticStatus__WARN);
    deadman_keyval = update_diagnostic_KeyValue(deadman_keyval, "On");
    digitalWrite(LED_PIN, HIGH);
  }
}

// Timer callback which publishes diaganostic status message at set interval
void diagnostic_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // check E stop
    if (digitalRead(estop_pin)) {
      teensy_status = update_diagnostic_status(
                        teensy_status,
                        "Motors deactivated",
                        diagnostic_msgs__msg__DiagnosticStatus__ERROR);
      estop = update_diagnostic_KeyValue(estop, "On");
    } else {
      estop = update_diagnostic_KeyValue(estop, "Off");
    }
    publish_diagnostics();
  }
}

// Timer callback which publishes encoder data message at set interval
void encoder_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);  // TO DO add Stale check here

  encoder_1_data.ticks = encoder_1.ticks;
  encoder_1_data.wheel_revs = encoder_1.total_revolutions;
  encoder_1_data.wheel_angle_deg = encoder_1.ticks_to_degrees();
  RCSOFTCHECK(rcl_publish(&enc_1_publisher, &encoder_1_data, NULL));

  encoder_2_data.ticks = encoder_2.ticks;
  encoder_2_data.wheel_revs = encoder_2.total_revolutions;
  encoder_2_data.wheel_angle_deg = encoder_2.ticks_to_degrees();
  RCSOFTCHECK(rcl_publish(&enc_2_publisher, &encoder_2_data, NULL));
  }

/**
* @brief Fills out the diagnostic message structure with the default values
*/
void init_diagnostics() {
  // Teensy Status
  teensy_status = create_diagnostic_status(teensy_status,
                    "Teensy Robot Driver",
                    "Init",
                    "3.2 5538870",
                    diagnostic_msgs__msg__DiagnosticStatus__OK);
  // Teensy Status Key-value pairs
  deadman_keyval = create_diagnostic_KeyValue(deadman_keyval, "Deadman Timer", "Init");
  estop = create_diagnostic_KeyValue(estop, "E Stop", "Off");
  side_lights = create_diagnostic_KeyValue(side_lights, "Side Lights", "Off");
  // add pairs to status in array
  diagnostic_msgs__msg__KeyValue__Sequence__init(&teensy_key_array, 3);
  teensy_key_array.data[0] = *deadman_keyval;
  teensy_key_array.data[1] = *estop;
  teensy_key_array.data[2] = *side_lights;
  teensy_status->values = teensy_key_array;

  // create other statuses
  motor_1_status = create_diagnostic_status(
                        motor_1_status,
                        motor_1.name,
                        "Awaiting Setup",
                        motor_1.hardware_id,
                        diagnostic_msgs__msg__DiagnosticStatus__WARN);

  motor_2_status = create_diagnostic_status(
                        motor_2_status,
                        motor_2.name,
                        "Awaiting Setup",
                        motor_2.hardware_id,
                        diagnostic_msgs__msg__DiagnosticStatus__WARN);

  encoder_1_status = create_diagnostic_status(
                          encoder_1_status,
                          encoder_1.name,
                          "Awaiting Setup",
                          encoder_1.hardware_id,
                          diagnostic_msgs__msg__DiagnosticStatus__WARN);

  encoder_2_status = create_diagnostic_status(
                          encoder_2_status,
                          encoder_2.name,
                          "Awaiting Setup",
                          encoder_2.hardware_id,
                          diagnostic_msgs__msg__DiagnosticStatus__WARN);
  battery_status = create_diagnostic_status(
                      battery_status,
                      "Battery",
                      "No monitoring",
                      "",
                      diagnostic_msgs__msg__DiagnosticStatus__WARN);

  // Fill diagnostic array with statuses
  diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(&status_array, 6);
  dia_array = diagnostic_msgs__msg__DiagnosticArray__create();
  status_array.data[0] = *teensy_status;
  status_array.data[1] = *motor_1_status;
  status_array.data[2] = *motor_2_status;
  status_array.data[3] = *encoder_1_status;
  status_array.data[4] = *encoder_2_status;
  status_array.data[5] = *battery_status;
  dia_array->status = status_array;
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(estop_pin, INPUT_PULLUP);

  // Neopixel startup
  ring_side.begin();           // Initialise NeoPixel strip object
  ring_side.show();            // Turn OFF all pixels ASAP
  ring_side.setBrightness(neo_side_bright);
  loading_chase(10, ring_side.Color(0, ring_side.gamma8(50), 0, 0), 15, &ring_side);
  ring_colour(pipebot_msgs__msg__Leds__COLOUR_GREEN, 50, &ring_side);
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  node_ops.domain_id = 20;
  RCCHECK(rclc_node_init_with_options(&node, "teensy_node", "", &support, &node_ops));

  // create service
  RCCHECK(rclc_service_init_default(
    &params_service,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(pipebot_msgs, srv, UpdateMicroConfig),
    "micro_params"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmd_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &led_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(pipebot_msgs, msg, Leds),
    "leds"));

  // create Diagnostic Status publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &diagnostics_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
    "diagnostics"));

  // create timer, to stop robot if no commands are recieved
  deadman_timer = rcl_get_zero_initialized_timer();
  RCCHECK(rclc_timer_init_default(
    &deadman_timer,
    &support,
    RCL_MS_TO_NS(deadman_timeout_ms),
    deadman_timer_callback));

  // create timer, to pub diagnostics at 1hz
  diagnostic_timer = rcl_get_zero_initialized_timer();
  RCCHECK(rclc_timer_init_default(
    &diagnostic_timer,
    &support,
    RCL_MS_TO_NS(1000/diagnostic_frequency_hz),  // convert Hz to ms
    diagnostic_timer_callback));

  // create timer, to pub encoders at 20hz
  encoder_timer = rcl_get_zero_initialized_timer();
  RCCHECK(rclc_timer_init_default(
    &encoder_timer,
    &support,
    RCL_MS_TO_NS(1000/encoder_frequency_hz),  // convert Hz to ms
    encoder_timer_callback));

  // create encoder publisher
  RCCHECK(rclc_publisher_init_best_effort(
  &enc_1_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(pipebot_msgs, msg, Encoders),
  "encoder_right"));

  // create encoder publisher
  RCCHECK(rclc_publisher_init_best_effort(
  &enc_2_publisher,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(pipebot_msgs, msg, Encoders),
  "encoder_left"));

  // create executor
  // total number of handles = #subscriptions + #timers + #services + #clients
  unsigned int num_handles = 2 + 3 + 1;
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));

  unsigned int rcl_wait_timeout = 1000;   // in ms
  RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_subscriber, &cmd_twist,
                                         &vel_received_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &led_subscriber, &led_data,
                                         &led_received_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &deadman_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &diagnostic_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &encoder_timer));
  RCCHECK(rclc_executor_add_service(&executor, &params_service, &request,
                                    &response, params_service_callback));


  init_diagnostics();

  // init motors
  if (motor_1.setup(d1_pin_speed, d1_pin_dir, d1_deadzone, d1_inverse) == true) {
    // Setup sucessful
    motor_1_status = update_diagnostic_status(
                        motor_1_status,
                        "Initialised",
                        diagnostic_msgs__msg__DiagnosticStatus__OK);
    publish_diagnostics();
  } else {
      motor_1_status = update_diagnostic_status(
                          motor_1_status,
                          "Type and pin mismatch",
                          diagnostic_msgs__msg__DiagnosticStatus__ERROR);
      publish_diagnostics();
      // block program and flash onboard LED
      while (1) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(500);
      }
  }
  if (motor_2.setup(d2_pin_speed, d2_pin_dir, d2_deadzone, d2_inverse) == true) {
    // setup sucessful
    motor_2_status = update_diagnostic_status(
                        motor_2_status,
                        "Initialised",
                        diagnostic_msgs__msg__DiagnosticStatus__OK);
    publish_diagnostics();
  } else {
    motor_2_status = update_diagnostic_status(
                        motor_2_status,
                        "Type and pin mismatch",
                        diagnostic_msgs__msg__DiagnosticStatus__ERROR);
    publish_diagnostics();
    // block program and flash onboard LED
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }

  // init encoders
  if (encoder_1.setup() == true) {
    // Setup sucessful
    encoder_1_status = update_diagnostic_status(
                          encoder_1_status,
                          "Initialised",
                           diagnostic_msgs__msg__DiagnosticStatus__OK);
    publish_diagnostics();
  } else {
      encoder_1_status = update_diagnostic_status(
                            encoder_1_status,
                            "Too many instances",
                            diagnostic_msgs__msg__DiagnosticStatus__ERROR);
      publish_diagnostics();
  }

  if (encoder_2.setup() == true) {
    // Setup sucessful
    encoder_2_status = update_diagnostic_status(
                            encoder_2_status,
                            "Initialised",
                            diagnostic_msgs__msg__DiagnosticStatus__OK);
    publish_diagnostics();
  } else {
      encoder_2_status = update_diagnostic_status(
                            encoder_2_status,
                            "Too many instances",
                            diagnostic_msgs__msg__DiagnosticStatus__ERROR);
      publish_diagnostics();
  }
}  // end setup

void loop() {
//  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
