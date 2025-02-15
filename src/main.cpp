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

// Speeds
int speedFB = 0; // Forward/Backward speed (0 to 200)
int speedLR = 0; // Left/Right speed (0 to 200)
int trakEnable=0;

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
  // Serial.begin(115200);
  Serial.begin(9600);
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

  // Wait for the serial connection to initialize
  while (!Serial) {
    ; 
  }

  // Add a blank line between the initialization nonsense output
  Serial.println("");
  Serial.println("Ready.");
}

// Forward declare for now to make the loop function easier to find
void parseCommand(String);
void motorUpdate(int speedFB, int speedLR);
void motorStop();

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming command
    String input = Serial.readStringUntil('\n');  // Read until newline

    parseCommand(input);
    motorUpdate(speedFB, speedLR);
  }

  // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // dataHandling(motor_command_data, leftMotors, rightMotors);
}

void parseCommand(String input) { //Pulls out the forard speed and left right speed 
  // Expected input format: "FB:100 LR:50 EN:1"   (At the moment EN reffers to traction Control only)

  int fbIndex = input.indexOf("FB:");
  int lrIndex = input.indexOf("LR:");
  int enIndex = input.indexOf("EN:"); //Enable switch for track module

  if (fbIndex != -1) {
    int fbEnd = input.indexOf(' ', fbIndex);
    if (fbEnd == -1) fbEnd = input.length();
    String fbValue = input.substring(fbIndex + 3, fbEnd);
    speedFB = fbValue.toInt();
  }

  if (lrIndex != -1) {
    int lrEnd = input.indexOf(' ', lrIndex);
    if (lrEnd == -1) lrEnd = input.length();
    String lrValue = input.substring(lrIndex + 3, lrEnd);
    speedLR = lrValue.toInt();
  }

  if (enIndex != -1) {
    int enEnd = input.indexOf(' ', enIndex);
    if (enEnd == -1) enEnd = input.length();
    String enValue = input.substring(enIndex + 3, enEnd);
    trakEnable = enValue.toInt();
  }

} 


void motorUpdate(int speedFB, int speedLR) {
    // Normalize input speeds from 0-200 to -100 to 100
    double speedFB_normalized = speedFB - 100;
    double speedLR_normalized = speedLR - 100;

    // Calculate raw motor speeds
    double leftSpeed = speedFB_normalized - speedLR_normalized;
    double rightSpeed = speedFB_normalized + speedLR_normalized;


    // Constrain motor speeds to valid range (-100 to 100)
    leftSpeed = constrain(leftSpeed, -100, 100);
    rightSpeed = constrain(rightSpeed, -100, 100);


    // Map motor speeds from -100 to 100 to -1.0 to 1.0
    leftSpeed = leftSpeed / 100.0;
    rightSpeed = rightSpeed / 100.0;

    // rightSpeed = 1.0;
    // leftSpeed = 1.0;

    if (rightSpeed > 0) {
        rightMotors->driveForwards(rightSpeed);
    } else {
        rightMotors->driveBackwards(rightSpeed);
    }
    if (leftSpeed > 0) {
        leftMotors->driveForwards(leftSpeed);
    } else {
        leftMotors->driveBackwards(leftSpeed);
    }

    // Debugging output
    Serial.print("Left Speed: ");
    Serial.print(leftSpeed);
    Serial.print(" Right Speed: ");
    Serial.println(rightSpeed);
}

void motorStop() { //Stops the Motor
  for (int i = 0; i < 3; i++) {
    rightMotors[i].stop();
    leftMotors[i].stop();
  }
  Serial.println("Motors Stopped");
}
