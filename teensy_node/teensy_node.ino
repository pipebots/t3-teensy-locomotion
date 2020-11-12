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

#include <std_msgs/msg/float64.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <geometry_msgs/msg/twist.h>

#include "config.h"

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_publisher_t status_publisher;

geometry_msgs__msg__Twist msg;
std_msgs__msg__Float64 floatmsg;
diagnostic_msgs__msg__DiagnosticArray dia_array;
diagnostic_msgs__msg__DiagnosticStatus * robot_status;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t deadman_timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// max_speed_ in m/s
// wheel_base_ in m
class RobotDriver{
    float max_speed_, wheel_base_;
  public:
    RobotDriver(float max_speed,float wheel_base) : max_speed_(max_speed), wheel_base_(wheel_base) {}
    void wheel_speeds(float, float);
    int percent_speed(float);
    float right_speed;
    float left_speed;
};

// Convert twist velocities to wheel speed for diff drive
void RobotDriver::wheel_speeds(float linear, float angular){
  left_speed = linear - angular*wheel_base_/2;
  right_speed = linear + angular*wheel_base_/2;

}

// While no vel feedback just convert desired velocity to percentage of robot max
int RobotDriver::percent_speed(float speed){
  int percent = (100 * speed/max_speed_);
  return percent;
}

// TODO: currently for half SN754410 H-Bridge, need to generalise
class Motor{
    int pin_en_, pin_A_, pin_B_, deadzone_;
  public:
    void setup(int, int, int, int);
    void move(int);
};

void Motor::setup(int enable, int pin_1, int pin_2, int dead_zone){
  pin_en_ = enable;
  pin_A_ = pin_1;
  pin_B_ = pin_2;
  deadzone_ = dead_zone;

  // set digital i/o pins as outputs:
  pinMode(pin_en_, OUTPUT);
  pinMode(pin_A_, OUTPUT);
  pinMode(pin_B_, OUTPUT);
}

// convert percentage speed to pwm and send to motors
void Motor::move(int percent_speed){
  int abs_speed = abs(percent_speed);
  abs_speed = constrain(abs_speed, 0, 100); //check and constrain to 0-100%

  //add deadzone to turn off motors if PID is close to 0.
  if (abs_speed < deadzone_){
    abs_speed = 0;
  }

  //conver % to pwm output
  int pwm = map(abs_speed, 0, 100, 0, 255);

  // set direction
  if(percent_speed >= 0){
    digitalWrite(pin_A_, HIGH);
    digitalWrite(pin_B_, LOW);
  }
  else{
    digitalWrite(pin_A_, LOW);
    digitalWrite(pin_B_, HIGH);

  }
  // set speed
  analogWrite(pin_en_, pwm);
  //floatmsg.data = pwm;
}

RobotDriver robot(max_speed, wheel_base); //TODO set as parameter externally
Motor left_motor;
Motor right_motor;

//flash for 5 second then restart in attempt to reconnect
void error_loop(){
  //stop motors
  left_motor.move(0);
  right_motor.move(0);
  for(int i = 0; i <=50; i++){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
  SCB_AIRCR = 0x05FA0004; //restart teensy to try to reconnect
}

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
  left_motor.move(l_percent_speed);
  floatmsg.data = l_percent_speed;
  RCSOFTCHECK(rcl_publish(&publisher, &floatmsg, NULL));
  right_motor.move(r_percent_speed);

}

// If no commands are recieved this executes and sets motors to 0
void deadman_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    left_motor.move(0);
    right_motor.move(0);
    floatmsg.data = 999;
    digitalWrite(LED_PIN, HIGH);
  }
}

/**
* @brief Fills out the diagnostic message structure with the defualt values
*/
void init_debug(){
  robot_status =   diagnostic_msgs__msg__DiagnosticStatus__create();
  diagnostic_msgs__msg__DiagnosticStatus__init(robot_status);
  //char bot_name = "Mobile Base";
  //rosidl_generator_c__DiagnosticStatus__assign(robot_status->name,bot_name, sizeof(bot_name));
//  robot_status->name = rosidl_generator_c("Mobile Base");
  //robot_status.level = diagnostic_msgs::DiagnosticStatus::OK;
  /*  robot_status.message = "Everything seem to be ok.";
    diagnostic_msgs::KeyValue emergency;
    emergency.key = "Emgergencystop hit";
    emergency.value = "false";
    diagnostic_msgs::KeyValue exited_normally;
    emergency.key = "Exited normally";
    emergency.value = "true";

    robot_status.values.push_back(emergency);
    robot_status.values.push_back(exited_normally);

    diagnostic_msgs::DiagnosticStatus  eth_status;
    eth_status.name = "EtherCAT Master";
    eth_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    eth_status.message = "Running";

    dia_array.status.push_back(robot_status);
    dia_array.status.push_back(eth_status);
  */
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus),
    "diagnostics"));

  // create timer, to stop robot if no commands are recieved
  deadman_timer = rcl_get_zero_initialized_timer();
  const unsigned int deadman_timeout = 500;
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
