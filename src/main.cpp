/*
Title: main.cpp
Author: Nick Linthacum
Date Created: 2/2/23
Date Modified: 3/3/24
Modification: works with Damon's GUI
Description: microros implementation for motor control.
*/
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#include "CPPGPIO/CPPGPIO.h"
#include "DataProcess/DataProcess.h"
#include "MotorSet/MotorSet.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

MotorSet* leftMotors = nullptr;
MotorSet* rightMotors = nullptr;

rcl_subscription_t left_motor_subscriber;
rcl_subscription_t right_motor_subscriber;

std_msgs__msg__Float32 left_motor_msg;
std_msgs__msg__Float32 right_motor_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float motor_command_data[2] = {0.0, 0.0};
#define LEFT_motor_TOPIC "motor_command/left_motor"
#define RIGHT_motor_TOPIC "motor_command/right_motor"

#define LED_PIN 26

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  // while(1) {
  //   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  //   delay(100);
  // }
}

void left_motor_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  motor_command_data[0] = msg->data;
}

void right_motor_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  motor_command_data[1] = msg->data;
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(LED_PIN, OUTPUT);
  
  delay(1000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "Motor_Microcontroller", "", &support));

  // create left motor subscriber
  RCCHECK(rclc_subscription_init_default(
    &left_motor_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    LEFT_motor_TOPIC));

  // create right motor subscriber
  RCCHECK(rclc_subscription_init_default(
    &right_motor_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    RIGHT_motor_TOPIC));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &left_motor_subscriber, 
    &left_motor_msg, 
    &left_motor_subscription_callback, 
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &right_motor_subscriber, 
    &right_motor_msg, 
    &right_motor_subscription_callback, 
    ON_NEW_DATA));

leftMotors = addPinsToLeftMotorSet();
rightMotors = addPinsToRightMotorSet();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  dataHandling(motor_command_data, leftMotors, rightMotors);
}