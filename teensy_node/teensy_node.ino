/* This is for a differential drive robot base with a teensy 3.2.
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

#include "robot_driver.h"
#include "motor.h"
#include "config.h"
#include "errors.h"

#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/string.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_publisher_t status_publisher;

geometry_msgs__msg__Twist msg;
std_msgs__msg__Float64 floatmsg;
diagnostic_msgs__msg__KeyValue deadman_keyval;
std_msgs__msg__String string_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t deadman_timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){coms_error(left_motor, right_motor);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){coms_error(left_motor, right_motor);}}

RobotDriver robot(max_speed, wheel_base); //TODO set as parameter externally
Motor left_motor(left_driver);
Motor right_motor(right_driver);

//void vel_received_callback(const geometry_msgs__msg__Twist msg) //const void * msgin)
void vel_received_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  digitalWrite(LED_PIN, LOW);
  //reset deadman_timer
  rcl_timer_reset(&deadman_timer);

  // extract vels from twist message
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  //calc desired wheel speed (m/s)
  robot.wheel_speeds(linear, angular);

  // get as percentage of max speed
  // (once there is velocity feedback we can do that instead)
  int l_percent_speed = robot.percent_speed(robot.left_speed);
  int r_percent_speed = robot.percent_speed(robot.right_speed);

  //move motors
  left_motor.move_percent(l_percent_speed);
  right_motor.move_percent(r_percent_speed);
  floatmsg.data = l_percent_speed;
  RCSOFTCHECK(rcl_publish(&publisher, &floatmsg, NULL));
  snprintf(deadman_keyval.value.data, deadman_keyval.value.capacity, "Off");
  RCSOFTCHECK(rcl_publish(&status_publisher, &deadman_keyval, NULL));
}

// If no commands are recieved this executes and sets motors to 0
void deadman_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    left_motor.move_percent(0);
    right_motor.move_percent(0);
    floatmsg.data = 999;
    snprintf(deadman_keyval.value.data, deadman_keyval.value.capacity, "Triggered");
    RCSOFTCHECK(rcl_publish(&publisher, &floatmsg, NULL));
    RCSOFTCHECK(rcl_publish(&status_publisher, &deadman_keyval, NULL));
    digitalWrite(LED_PIN, HIGH);
  }
}

/**
* @brief Fills out the diagnostic message structure with the defualt values
*/
void init_debug(){
  //robot_status =   diagnostic_msgs__msg__DiagnosticStatus__create();
  //key_msg = diagnostic_msgs__msg__KeyValue__create();
  std_msgs__msg__String__init(&string_msg); //if i remove this the comms is unreliable?! must me some memory initilasation thing?
  diagnostic_msgs__msg__KeyValue__init(&deadman_keyval);
  const unsigned int KEY_SIZE = 20;
  deadman_keyval.key.capacity = KEY_SIZE;
  snprintf(deadman_keyval.key.data, deadman_keyval.key.capacity, "Deadman Timer");
  deadman_keyval.key.size = strlen(deadman_keyval.key.data);

  deadman_keyval.value.capacity = KEY_SIZE;
  snprintf(deadman_keyval.value.data, deadman_keyval.value.capacity, "Initilised");
  deadman_keyval.value.size = strlen(deadman_keyval.value.data);
}

void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  left_motor.setup(left_pin_en, left_pin_a, left_pin_b, left_deadzone);
  right_motor.setup(right_pin_en, right_pin_a, right_pin_b, right_deadzone);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  node_ops.domain_id = 20;
  RCCHECK(rclc_node_init_with_options(&node, "micro_ros_arduino_node", "", &support, &node_ops));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create publisher - currently for debug
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "micro_ros_arduino_node_publisher"));

  // create Diagnostic Status publisher
  RCCHECK(rclc_publisher_init_default(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, KeyValue),
    "diagnostics"));

  // create timer, to stop robot if no commands are recieved
  deadman_timer = rcl_get_zero_initialized_timer();
  RCCHECK(rclc_timer_init_default(
    &deadman_timer,
    &support,
    RCL_MS_TO_NS(deadman_timeout),
    deadman_timer_callback));

  // create executor
  // total number of handles = #subscriptions + #timers
  unsigned int num_handles = 1 + 2;
  executor = rclc_executor_get_zero_initialized_executor();
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));

  unsigned int rcl_wait_timeout = 1000;   // in ms
  RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &vel_received_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &deadman_timer));

  init_debug();
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
