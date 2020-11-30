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

#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <geometry_msgs/msg/twist.h>


rcl_subscription_t cmd_subscriber;
rcl_publisher_t status_publisher;

geometry_msgs__msg__Twist cmd_twist;
diagnostic_msgs__msg__DiagnosticStatus status;
diagnostic_msgs__msg__KeyValue deadman_keyval;
diagnostic_msgs__msg__KeyValue estop;
diagnostic_msgs__msg__KeyValue__Sequence key_array;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t deadman_timer;
rcl_timer_t diagnostic_timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){coms_error(left_motor, right_motor);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){coms_error(left_motor, right_motor);}}

RobotDriver robot(max_speed, wheel_base);
Motor left_motor(left_driver);
Motor right_motor(right_driver);

void publish_status(){
  deadman_keyval.value.size = strlen(deadman_keyval.value.data);
  estop.value.size = strlen(estop.value.data);
  status.message.size = strlen(status.message.data);

  // update key value array
  key_array.data[0] = deadman_keyval;
  key_array.data[1] = estop;
  status.values = key_array;

  RCSOFTCHECK(rcl_publish(&status_publisher, &status, NULL));
}

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
  snprintf(deadman_keyval.value.data, deadman_keyval.value.capacity, "Off");
  snprintf(status.message.data, status.message.capacity, "messages recieved from /cmd_vel");
  status.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
}

// If no commands are recieved this executes and sets motors to 0
void deadman_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    left_motor.move_percent(0);
    right_motor.move_percent(0);

    snprintf(deadman_keyval.value.data, deadman_keyval.value.capacity, "Triggered");
    snprintf(status.message.data, status.message.capacity, "No messages recieved from /cmd_vel for 500ms");
    status.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
    digitalWrite(LED_PIN, HIGH);
  }
}

// Publish diaganostic status messages
void diagnostic_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  publish_status();
}

/**
* @brief Fills out the diagnostic message structure with the defualt values
*/
void init_debug(){
  // Init key_val message
  diagnostic_msgs__msg__KeyValue__init(&deadman_keyval);
  diagnostic_msgs__msg__KeyValue__init(&estop);

  // Init Dadman timer Key
  const unsigned int KEY_SIZE = 10;
  deadman_keyval.key.data = (char*)malloc(14*sizeof(char));
  deadman_keyval.key.size = 0;
  deadman_keyval.key.capacity = 14;
  // Use Key
  snprintf(deadman_keyval.key.data, deadman_keyval.key.capacity, "Deadman Timer");
  deadman_keyval.key.size = strlen(deadman_keyval.key.data);
  //Init Value
  deadman_keyval.value.data = (char*)malloc(4*sizeof(char));
  deadman_keyval.value.size = 0;
  deadman_keyval.value.capacity = KEY_SIZE;
  //Use Value
  snprintf(deadman_keyval.value.data, deadman_keyval.value.capacity, "Init");
  deadman_keyval.value.size = strlen(deadman_keyval.value.data);

  // Init E stop Key
  estop.key.data = (char*)malloc(15*sizeof(char));
  estop.key.size = 0;
  estop.key.capacity = 15;
  // Use Key
  snprintf(estop.key.data, estop.key.capacity, "Emergency Stop");
  estop.key.size = strlen(estop.key.data);
  //Init Value
  estop.value.data = (char*)malloc(4*sizeof(char));
  estop.value.size = 0;
  estop.value.capacity = 4;
  //Use Value
  snprintf(estop.value.data, estop.value.capacity, "Off");
  estop.value.size = strlen(estop.value.data);

  diagnostic_msgs__msg__DiagnosticStatus__init(&status);
  status.level = diagnostic_msgs__msg__DiagnosticStatus__OK;

  status.name.data = (char*)malloc(20*sizeof(char));
  status.name.capacity = 20;
  snprintf(status.name.data, status.name.capacity, "Teensy Robot Driver");
  status.name.size = strlen(status.name.data);

  status.message.data = (char*)malloc(100*sizeof(char));
  status.message.capacity = 100;
  snprintf(status.message.data, status.message.capacity, "Initialised A-OKAY!");
  status.message.size = strlen(status.message.data);

  status.hardware_id.data = (char*)malloc(10*sizeof(char));
  status.hardware_id.capacity = 10;
  snprintf(status.hardware_id.data, status.hardware_id.capacity, "teensy3.2");
  status.hardware_id.size = strlen(status.hardware_id.data);

  diagnostic_msgs__msg__KeyValue__Sequence__init(&key_array, 2);
  key_array.data[0] = deadman_keyval;
  key_array.data[1] = estop;
  status.values = key_array;
}

void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
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
    &status_publisher,
    &node,
    //ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, KeyValue),
    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus),
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
    RCL_MS_TO_NS(1000/diagnostic_frequency), // convert Hz to ms
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


  init_debug();

  // init motors
  if (left_motor.setup(left_pin_en, left_pin_a, left_pin_b, left_deadzone) == true) {
    // Setup sucessful
    snprintf(status.message.data, status.message.capacity, "Left Motor Setup");
    status.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
    publish_status();
  } else {
      snprintf(status.message.data, status.message.capacity, "Error: Left Motor - Driver type and number of pins initialised do not match");
      status.level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
      publish_status();
      // block program and flash onboard LED
      while (1) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(500);
      }
  }
  if (right_motor.setup(right_pin_en, right_pin_a, right_pin_b, right_deadzone) == true){
    // setup sucessful
    snprintf(status.message.data, status.message.capacity, "Right Motor Setup");
    status.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
    publish_status();
  } else {
    snprintf(status.message.data, status.message.capacity, "Error: Right Motor - Driver type and number of pins initialised do not match");
    status.level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
    publish_status();
    // block program and flash onboard LED
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(500);
    }
  }
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
