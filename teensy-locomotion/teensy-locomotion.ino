/*
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
*  (c) Nick Fry 2020
*/

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <geometry_msgs/msg/twist.h>

#include "robot_driver.h"
#include "motor.h"
#include "config.h"
#include "diagnostics.h"
#include "encoder.h"




rcl_subscription_t cmd_subscriber;
rcl_publisher_t diagnostics_publisher;

geometry_msgs__msg__Twist cmd_twist;
diagnostic_msgs__msg__DiagnosticStatus * teensy_status;
diagnostic_msgs__msg__DiagnosticStatus * motor_1_status;
diagnostic_msgs__msg__DiagnosticStatus * motor_2_status;
diagnostic_msgs__msg__DiagnosticStatus * encoder_1_status;
diagnostic_msgs__msg__DiagnosticStatus * encoder_2_status;
diagnostic_msgs__msg__DiagnosticStatus * battery_status;
diagnostic_msgs__msg__KeyValue * deadman_keyval;
diagnostic_msgs__msg__KeyValue * estop;
diagnostic_msgs__msg__KeyValue * headlights;
diagnostic_msgs__msg__KeyValue__Sequence teensy_key_array;
diagnostic_msgs__msg__DiagnosticStatus__Sequence status_array;
diagnostic_msgs__msg__DiagnosticArray * dia_array;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t deadman_timer;
rcl_timer_t diagnostic_timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){coms_error(&motor_1, &motor_2, LED_PIN);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){coms_error(&motor_1, &motor_2, LED_PIN);}}

RobotDriver robot(max_speed, wheel_base);
Motor motor_1(driver_1_name, driver_1_id, driver_1);
Motor motor_2(driver_2_name, driver_2_id, driver_2);
Encoder encoder_1(encoder_1_name, en1_pin_a, en1_pin_b,
                  en1_counts_per_rev, encoder_1_id, en1_inverse);
Encoder encoder_2(encoder_2_name, en2_pin_a, en2_pin_b,
                  en2_counts_per_rev, encoder_2_id, en2_inverse);

void publish_diagnostics() {
  // update key value array
  teensy_key_array.data[0] = *deadman_keyval;
  teensy_key_array.data[1] = *estop;
  teensy_key_array.data[2] = *headlights;
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
  deadman_keyval = update_diagnostic_KeyValue(deadman_keyval, "Off");
  teensy_status = update_diagnostic_status(
                    teensy_status,
                    "messages recieved from /cmd_vel",
                    diagnostic_msgs__msg__DiagnosticStatus__OK);
}

// If no commands are recieved this executes and sets motors to 0
void deadman_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    motor_1.move_percent(0);
    motor_2.move_percent(0);

    teensy_status = update_diagnostic_status(
                      teensy_status,
                      "No messages recieved from /cmd_vel for 500ms",
                      diagnostic_msgs__msg__DiagnosticStatus__WARN);
    deadman_keyval = update_diagnostic_KeyValue(deadman_keyval, "Triggered");
    digitalWrite(LED_PIN, HIGH);
  }
}

// Timer callback which publishes diaganostic status message at set interval
void diagnostic_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  publish_diagnostics();
}

/**
* @brief Fills out the diagnostic message structure with the default values
*/
void init_diagnostics() {
  // Teensy Status
  teensy_status = create_diagnostic_status(teensy_status,
                    "Teensy Robot Driver",
                    "Init",
                    "teensy3.2",
                    diagnostic_msgs__msg__DiagnosticStatus__OK);
  // Teensy Status Key-value pairs
  deadman_keyval = create_diagnostic_KeyValue(deadman_keyval, "Deadman Timer", "Init");
  estop = create_diagnostic_KeyValue(deadman_keyval, "Emergency Stop", "Off");
  headlights = create_diagnostic_KeyValue(deadman_keyval, "Headlights", "Off");
  // add pairs to status in array
  diagnostic_msgs__msg__KeyValue__Sequence__init(&teensy_key_array, 3);
  teensy_key_array.data[0] = *deadman_keyval;
  teensy_key_array.data[1] = *estop;
  teensy_key_array.data[2] = *headlights;
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
                      "No monitoring implemented",
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
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  node_ops.domain_id = 20;
  RCCHECK(rclc_node_init_with_options(&node, "teensy_node", "", &support, &node_ops));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmd_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create Diagnostic Status publisher
  RCCHECK(rclc_publisher_init_default(
  // RCCHECK(rclc_publisher_init_best_effort(
    &diagnostics_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
    "diagnostics"));

  // create timer, to stop robot if no commands are recieved
  deadman_timer = rcl_get_zero_initialized_timer();
  RCCHECK(rclc_timer_init_default(
    &deadman_timer,
    &support,
    RCL_MS_TO_NS(deadman_timeout),
    deadman_timer_callback));

  // create timer, to pub diagnostics at 1hz
  diagnostic_timer = rcl_get_zero_initialized_timer();
  RCCHECK(rclc_timer_init_default(
    &diagnostic_timer,
    &support,
    RCL_MS_TO_NS(1000/diagnostic_frequency),  // convert Hz to ms
    diagnostic_timer_callback));

  // create executor
  // total number of handles = #subscriptions + #timers
  unsigned int num_handles = 1 + 2;
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));

  unsigned int rcl_wait_timeout = 1000;   // in ms
  RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_subscriber, &cmd_twist, &vel_received_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &deadman_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &diagnostic_timer));

  init_diagnostics();

  // init motors
  if (motor_1.setup(d1_pin_en, d1_pin_a, d1_pin_b, d1_deadzone) == true) {
    // Setup sucessful
    motor_1_status = update_diagnostic_status(
                          motor_1_status,
                          "Initialised",
                           diagnostic_msgs__msg__DiagnosticStatus__OK);
    publish_diagnostics();
  } else {
      motor_1_status = update_diagnostic_status(
                            motor_1_status,
                            "Error: Driver type and number of pins initialised do not match",
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
  if (motor_2.setup(d2_pin_en, d2_pin_a, d2_pin_b, d2_deadzone) == true) {
    // setup sucessful
    motor_2_status = update_diagnostic_status(
                          motor_2_status,
                          "Initialised",
                          diagnostic_msgs__msg__DiagnosticStatus__OK);
    publish_diagnostics();
  } else {
    motor_2_status = update_diagnostic_status(
                          motor_2_status,
                          "Error: Driver type and number of pins initialised do not match",
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
                              "Error: Encoder not initialised, too many instances.",
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
                              "Error: Encoder not initialised, too many instances.",
                              diagnostic_msgs__msg__DiagnosticStatus__ERROR);
      publish_diagnostics();
  }

}  // end setup

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
