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

#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>
#include <geometry_msgs/msg/twist.h>


rcl_subscription_t cmd_subscriber;
rcl_publisher_t diagnostics_publisher;

geometry_msgs__msg__Twist cmd_twist;
diagnostic_msgs__msg__DiagnosticStatus teensy_status;
diagnostic_msgs__msg__DiagnosticStatus left_motor_status;
diagnostic_msgs__msg__DiagnosticStatus right_motor_status;
diagnostic_msgs__msg__DiagnosticStatus left_encoder_status;
diagnostic_msgs__msg__DiagnosticStatus right_encoder_status;
diagnostic_msgs__msg__DiagnosticStatus battery_status;
diagnostic_msgs__msg__KeyValue deadman_keyval;
diagnostic_msgs__msg__KeyValue estop;
diagnostic_msgs__msg__KeyValue headlights;
diagnostic_msgs__msg__KeyValue__Sequence teensy_key_array;
diagnostic_msgs__msg__DiagnosticStatus__Sequence status_array;
diagnostic_msgs__msg__DiagnosticArray dia_array;

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

void publish_diagnostics(){

  // ensure size allocations are correct
  // (safer to do all here but less efficient...does it matter?)
  deadman_keyval.value.size = strlen(deadman_keyval.value.data);
  estop.value.size = strlen(estop.value.data);
  headlights.value.size = strlen(headlights.value.data);
  teensy_status.message.size = strlen(teensy_status.message.data);
  left_motor_status.message.size = strlen(left_motor_status.message.data);
  right_motor_status.message.size = strlen(right_motor_status.message.data);
  left_encoder_status.message.size = strlen(left_encoder_status.message.data);
  right_encoder_status.message.size = strlen(right_encoder_status.message.data);
  battery_status.message.size = strlen(battery_status.message.data);

  // update key value array
  teensy_key_array.data[0] = deadman_keyval;
  teensy_key_array.data[1] = estop;
  teensy_key_array.data[2] = headlights;
  teensy_status.values = teensy_key_array;

  // update status array
  status_array.data[0] = teensy_status;
  status_array.data[1] = left_motor_status;
  status_array.data[2] = right_motor_status;
  status_array.data[3] = left_encoder_status;
  status_array.data[4] = right_encoder_status;
  status_array.data[5] = battery_status;
  dia_array.status = status_array;

  RCSOFTCHECK(rcl_publish(&diagnostics_publisher, &dia_array, NULL));
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
  snprintf(teensy_status.message.data, teensy_status.message.capacity, "messages recieved from /cmd_vel");
  teensy_status.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
}

// If no commands are recieved this executes and sets motors to 0
void deadman_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    left_motor.move_percent(0);
    right_motor.move_percent(0);

    snprintf(deadman_keyval.value.data, deadman_keyval.value.capacity, "Triggered");
    snprintf(teensy_status.message.data, teensy_status.message.capacity, "No messages recieved from /cmd_vel for 500ms");
    teensy_status.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
    digitalWrite(LED_PIN, HIGH);
  }
}

// Timer callback which publishes diaganostic status message at set interval
void diagnostic_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  publish_diagnostics();
}

/**
* @brief Fills out the diagnostic message structure with the defualt values
*/
void init_diagnostics(){
  //---------Teensy Status------------------
  diagnostic_msgs__msg__DiagnosticStatus__init(&teensy_status);
  teensy_status.level = diagnostic_msgs__msg__DiagnosticStatus__OK;

  teensy_status.name.data = (char*)malloc(20*sizeof(char));
  teensy_status.name.capacity = 20;
  snprintf(teensy_status.name.data, teensy_status.name.capacity, "Teensy Robot Driver");
  teensy_status.name.size = strlen(teensy_status.name.data);

  teensy_status.message.data = (char*)malloc(100*sizeof(char));
  teensy_status.message.capacity = 100;
  snprintf(teensy_status.message.data, teensy_status.message.capacity, "Initialised A-OKAY!");
  teensy_status.message.size = strlen(teensy_status.message.data);

  teensy_status.hardware_id.data = (char*)malloc(10*sizeof(char));
  teensy_status.hardware_id.capacity = 10;
  snprintf(teensy_status.hardware_id.data, teensy_status.hardware_id.capacity, "teensy3.2");
  teensy_status.hardware_id.size = strlen(teensy_status.hardware_id.data);

  // Init key_val messages
  diagnostic_msgs__msg__KeyValue__init(&deadman_keyval);
  diagnostic_msgs__msg__KeyValue__init(&estop);
  diagnostic_msgs__msg__KeyValue__init(&headlights);

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

  // Init LED Key
  headlights.key.data = (char*)malloc(11*sizeof(char));
  headlights.key.size = 0;
  headlights.key.capacity = 11;
  // Use Key
  snprintf(headlights.key.data, headlights.key.capacity, "Headlights");
  headlights.key.size = strlen(headlights.key.data);
  //Init Value
  headlights.value.data = (char*)malloc(4*sizeof(char));
  headlights.value.size = 0;
  headlights.value.capacity = 4;
  //Use Value
  snprintf(headlights.value.data, headlights.value.capacity, "Off");
  headlights.value.size = strlen(headlights.value.data);

  diagnostic_msgs__msg__KeyValue__Sequence__init(&teensy_key_array, 3);
  teensy_key_array.data[0] = deadman_keyval;
  teensy_key_array.data[1] = estop;
  teensy_key_array.data[2] = headlights;
  teensy_status.values = teensy_key_array;

  //-----------Left Motor Status----------------------
  diagnostic_msgs__msg__DiagnosticStatus__init(&left_motor_status);
  left_motor_status.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;

  left_motor_status.name.data = (char*)malloc(18*sizeof(char));
  left_motor_status.name.capacity = 18;
  snprintf(left_motor_status.name.data, left_motor_status.name.capacity, "Left Motor Driver");
  left_motor_status.name.size = strlen(left_motor_status.name.data);

  left_motor_status.message.data = (char*)malloc(100*sizeof(char));
  left_motor_status.message.capacity = 100;
  snprintf(left_motor_status.message.data, left_motor_status.message.capacity, "Awaiting Setup");
  left_motor_status.message.size = strlen(left_motor_status.message.data);

  left_motor_status.hardware_id.data = (char*)malloc(14*sizeof(char));
  left_motor_status.hardware_id.capacity = 14;
  snprintf(left_motor_status.hardware_id.data, left_motor_status.hardware_id.capacity, "left_SN754410");
  left_motor_status.hardware_id.size = strlen(left_motor_status.hardware_id.data);

  //-----------Right Motor Status----------------------
  diagnostic_msgs__msg__DiagnosticStatus__init(&right_motor_status);
  right_motor_status.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;

  right_motor_status.name.data = (char*)malloc(19*sizeof(char));
  right_motor_status.name.capacity = 19;
  snprintf(right_motor_status.name.data, right_motor_status.name.capacity, "Right Motor Driver");
  right_motor_status.name.size = strlen(right_motor_status.name.data);

  right_motor_status.message.data = (char*)malloc(100*sizeof(char));
  right_motor_status.message.capacity = 100;
  snprintf(right_motor_status.message.data, right_motor_status.message.capacity, "Awaiting Setup");
  right_motor_status.message.size = strlen(right_motor_status.message.data);

  right_motor_status.hardware_id.data = (char*)malloc(15*sizeof(char));
  right_motor_status.hardware_id.capacity = 15;
  snprintf(right_motor_status.hardware_id.data, right_motor_status.hardware_id.capacity, "right_SN754410");
  right_motor_status.hardware_id.size = strlen(right_motor_status.hardware_id.data);

  //-----------Left Encoder Status----------------------
  diagnostic_msgs__msg__DiagnosticStatus__init(&left_encoder_status);
  left_encoder_status.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;

  left_encoder_status.name.data = (char*)malloc(13*sizeof(char));
  left_encoder_status.name.capacity = 13;
  snprintf(left_encoder_status.name.data, left_encoder_status.name.capacity, "Left Encoder");
  left_encoder_status.name.size = strlen(left_encoder_status.name.data);

  left_encoder_status.message.data = (char*)malloc(100*sizeof(char));
  left_encoder_status.message.capacity = 100;
  snprintf(left_encoder_status.message.data, left_encoder_status.message.capacity, "No encoder implemented");
  left_encoder_status.message.size = strlen(left_encoder_status.message.data);

  left_encoder_status.hardware_id.data = (char*)malloc(15*sizeof(char));
  left_encoder_status.hardware_id.capacity = 15;
  snprintf(left_encoder_status.hardware_id.data, left_encoder_status.hardware_id.capacity, " ");
  left_encoder_status.hardware_id.size = strlen(left_encoder_status.hardware_id.data);

  //-----------Right Encoder Status----------------------
  diagnostic_msgs__msg__DiagnosticStatus__init(&right_encoder_status);
  right_encoder_status.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;

  right_encoder_status.name.data = (char*)malloc(13*sizeof(char));
  right_encoder_status.name.capacity = 13;
  snprintf(right_encoder_status.name.data, right_encoder_status.name.capacity, "Right Encoder");
  right_encoder_status.name.size = strlen(right_encoder_status.name.data);

  right_encoder_status.message.data = (char*)malloc(100*sizeof(char));
  right_encoder_status.message.capacity = 100;
  snprintf(right_encoder_status.message.data, right_encoder_status.message.capacity, "No encoder implemented");
  right_encoder_status.message.size = strlen(right_encoder_status.message.data);

  right_encoder_status.hardware_id.data = (char*)malloc(15*sizeof(char));
  right_encoder_status.hardware_id.capacity = 15;
  snprintf(right_encoder_status.hardware_id.data, right_encoder_status.hardware_id.capacity, " ");
  right_encoder_status.hardware_id.size = strlen(right_encoder_status.hardware_id.data);

  //-----------Battery Status----------------------
  diagnostic_msgs__msg__DiagnosticStatus__init(&battery_status);
  battery_status.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;

  battery_status.name.data = (char*)malloc(8*sizeof(char));
  battery_status.name.capacity = 8;
  snprintf(battery_status.name.data, battery_status.name.capacity, "Battery");
  battery_status.name.size = strlen(battery_status.name.data);

  battery_status.message.data = (char*)malloc(100*sizeof(char));
  battery_status.message.capacity = 100;
  snprintf(battery_status.message.data, battery_status.message.capacity, "No monitoring implemented");
  battery_status.message.size = strlen(battery_status.message.data);

  battery_status.hardware_id.data = (char*)malloc(15*sizeof(char));
  battery_status.hardware_id.capacity = 15;
  snprintf(battery_status.hardware_id.data, battery_status.hardware_id.capacity, " ");
  battery_status.hardware_id.size = strlen(battery_status.hardware_id.data);

  //---------Fill diagnostic array --------------------
  diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(&status_array, 6);
  diagnostic_msgs__msg__DiagnosticArray__init(&dia_array);
  status_array.data[0] = teensy_status;
  status_array.data[1] = left_motor_status;
  status_array.data[2] = right_motor_status;
  status_array.data[3] = left_encoder_status;
  status_array.data[4] = right_encoder_status;
  status_array.data[5] = battery_status;
  dia_array.status = status_array;
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
  //RCCHECK(rclc_publisher_init_best_effort(
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


  init_diagnostics();

  // init motors
  if (left_motor.setup(left_pin_en, left_pin_a, left_pin_b, left_deadzone) == true) {
    // Setup sucessful
    snprintf(left_motor_status.message.data, left_motor_status.message.capacity, "Initialised");
    left_motor_status.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
    publish_diagnostics();
  } else {
      snprintf(left_motor_status.message.data, left_motor_status.message.capacity, "Error: Driver type and number of pins initialised do not match");
      left_motor_status.level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
      publish_diagnostics();
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
    snprintf(right_motor_status.message.data, right_motor_status.message.capacity, "Initialised");
    right_motor_status.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
    publish_diagnostics();
  } else {
    snprintf(right_motor_status.message.data, right_motor_status.message.capacity, "Error: Driver type and number of pins initialised do not match");
    right_motor_status.level = diagnostic_msgs__msg__DiagnosticStatus__ERROR;
    publish_diagnostics();
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
