#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <diagnostic_msgs/msg/key_value.h>

rcl_publisher_t publisher;
rcl_publisher_t status_publisher;
std_msgs__msg__Int32 msg;
diagnostic_msgs__msg__KeyValue deadman_keyval;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t timer2;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void timer2_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&status_publisher, &deadman_keyval, NULL));
    char snum[5];
    itoa(msg.data, snum, 10);
    snprintf(deadman_keyval.value.data, deadman_keyval.value.capacity, snum);
    deadman_keyval.value.size = strlen(deadman_keyval.value.data);
  }
}

void init_debug(){

  diagnostic_msgs__msg__KeyValue__init(&deadman_keyval);
  const unsigned int KEY_SIZE = 20;
  deadman_keyval.key.data = (char*)malloc(KEY_SIZE*sizeof(char));
  deadman_keyval.key.size = 0;
  deadman_keyval.key.capacity = KEY_SIZE;

  snprintf(deadman_keyval.key.data, deadman_keyval.key.capacity, "Deadman Timer");
  deadman_keyval.key.size = strlen(deadman_keyval.key.data);

  deadman_keyval.value.data = (char*)malloc(KEY_SIZE*sizeof(char));
  deadman_keyval.value.size = 0;
  deadman_keyval.value.capacity = KEY_SIZE;

  snprintf(deadman_keyval.value.data, deadman_keyval.value.capacity, "Init");
  deadman_keyval.value.size = strlen(deadman_keyval.value.data);
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
  RCCHECK(rclc_node_init_with_options(&node, "micro_ros_arduino_node", "", &support, &node_ops));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));

  // create Diagnostic Status publisher
  RCCHECK(rclc_publisher_init_default(
    &status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, KeyValue),
  //  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "diagnostics"));

  // create timer,
  timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create timer2,
  timer2 = rcl_get_zero_initialized_timer();
  const unsigned int timer2_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer2,
    &support,
    RCL_MS_TO_NS(timer2_timeout),
    timer2_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  unsigned int rcl_wait_timeout = 100;   // in ms
  RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &timer2));
  msg.data = 0;
  init_debug();

}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
