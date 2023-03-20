//This code is for taking the single float version that works and trying an array
//implementing separate subscibers and my active worlspace from 3/5


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


rcl_subscription_t left_trigger_subscriber;
rcl_subscription_t right_trigger_subscriber;
rcl_subscription_t left_shoulder_subscriber;
rcl_subscription_t right_shoulder_subscriber;
rcl_subscription_t dpad_lr_subscriber;


std_msgs__msg__Float32 left_trigger_msg;
std_msgs__msg__Float32 right_trigger_msg;
std_msgs__msg__Float32 left_shoulder_msg;
std_msgs__msg__Float32 right_shoulder_msg;
std_msgs__msg__Float32 dpad_lr_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


float motor_command_data[5] = {0.99, 0.99, 0.0, 0.0, 0.0};
#define LEFT_TRIGGER_TOPIC "motor_command/left_trigger"
#define RIGHT_TRIGGER_TOPIC "motor_command/right_trigger"
#define LEFT_SHOULDER_TOPIC "motor_command/left_shoulder"
#define RIGHT_SHOULDER_TOPIC "motor_command/right_shoulder"
#define DPAD_LR_TOPIC "motor_command/dpad_lr"

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

void left_trigger_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  motor_command_data[0] = msg->data;


}

void right_trigger_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;


  motor_command_data[1] = msg->data;
}


void left_shoulder_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  motor_command_data[2] = msg->data;
}

void right_shoulder_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  motor_command_data[3] = msg->data;
}

void dpad_lr_subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

  motor_command_data[4] = msg->data;
}


void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);


  


  pinMode(LED_PIN, OUTPUT);
  

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "Motor_Microcontroller", "", &support));

  // create left trigger subscriber
  RCCHECK(rclc_subscription_init_default(
    &left_trigger_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    LEFT_TRIGGER_TOPIC));

  // create right trigger subscriber
  RCCHECK(rclc_subscription_init_default(
    &right_trigger_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    RIGHT_TRIGGER_TOPIC));


  // create left shoulder subscriber
  RCCHECK(rclc_subscription_init_default(
    &left_shoulder_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    LEFT_SHOULDER_TOPIC));

  // create right shoulder subscriber
  RCCHECK(rclc_subscription_init_default(
    &right_shoulder_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    RIGHT_SHOULDER_TOPIC));

  // create dpad_lr subscriber
  RCCHECK(rclc_subscription_init_default(
    &dpad_lr_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    DPAD_LR_TOPIC));








  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &left_trigger_subscriber, 
    &left_trigger_msg, 
    &left_trigger_subscription_callback, 
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &right_trigger_subscriber, 
    &right_trigger_msg, 
    &right_trigger_subscription_callback, 
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &left_shoulder_subscriber, 
    &left_shoulder_msg, 
    &left_shoulder_subscription_callback, 
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &right_shoulder_subscriber, 
    &right_shoulder_msg, 
    &right_shoulder_subscription_callback, 
    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &dpad_lr_subscriber, 
    &dpad_lr_msg, 
    &dpad_lr_subscription_callback, 
    ON_NEW_DATA));



leftMotors = addPinsToLeftMotorSet();
rightMotors = addPinsToRightMotorSet();
}

void loop() {


  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  dataHandling(motor_command_data, leftMotors, rightMotors);
  // digitalWrite(LED_PIN, HIGH); 
  // delay(100);
  // digitalWrite(LED_PIN, LOW); 
  // delay(100);
  //delay(10);


}





































































// #include <Arduino.h>
// #include <micro_ros_platformio.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/float32.h>

// #include "CPPGPIO/CPPGPIO.h"
// #include "DataProcess/DataProcess.h"
// #include "MotorSet/MotorSet.h"


// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only available for Arduino framework with serial transport.
// #endif


// MotorSet* leftMotors = nullptr;
// MotorSet* rightMotors = nullptr;


// rcl_subscription_t left_subscriber;
// rcl_subscription_t right_subscriber;
// std_msgs__msg__Float32 left_msg;
// std_msgs__msg__Float32 right_msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;


// float motor_command_data[5] = {0.99, 0.99, 0.0, 0.0, 0.0};
// #define LEFT_TRIGGER_TOPIC "motor_command/left_trigger"
// #define RIGHT_TRIGGER_TOPIC "motor_command/right_trigger"
// #define LEFT_LED_PIN 25
// #define RIGHT_LED_PIN 26

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// // Error handle loop
// void error_loop() {
//   // while(1) {
//   //   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//   //   delay(100);
//   // }
// }

// void left_subscription_callback(const void * msgin)
// {  
//   const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

//   motor_command_data[0] = msg->data;

//   if (msg->data > 0)
//   {
//     digitalWrite(LEFT_LED_PIN, HIGH);
//   }
//   else
//   {
//     digitalWrite(LEFT_LED_PIN, LOW);
//   }
// }

// void right_subscription_callback(const void * msgin)
// {  
//   const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

//   if (msg->data > 0)
//   {
//    digitalWrite(RIGHT_LED_PIN, HIGH);
//    }
//   else
//   {
//     digitalWrite(RIGHT_LED_PIN, LOW);
//  }

//   motor_command_data[1] = msg->data;
// }

// void setup() {
//   // Configure serial transport
//   Serial.begin(115200);
//   set_microros_serial_transports(Serial);


//   pinMode(LEFT_LED_PIN, OUTPUT);
//   pinMode(RIGHT_LED_PIN, OUTPUT);
//   // digitalWrite(LED_PIN, HIGH); 

//   delay(2000);

//   allocator = rcl_get_default_allocator();

//   //create init_options
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   // create node
//   RCCHECK(rclc_node_init_default(&node, "LEDreceiver", "", &support));

//   // create left trigger subscriber
//   RCCHECK(rclc_subscription_init_default(
//     &left_subscriber,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//     LEFT_TRIGGER_TOPIC));

//   // create right trigger subscriber
//   RCCHECK(rclc_subscription_init_default(
//     &right_subscriber,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//     RIGHT_TRIGGER_TOPIC));

//   // create executor
//   RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
//   RCCHECK(rclc_executor_add_subscription(
//     &executor, 
//     &left_subscriber, 
//     &left_msg, 
//     &left_subscription_callback, 
//     ON_NEW_DATA));
//   RCCHECK(rclc_executor_add_subscription(
//     &executor, 
//     &right_subscriber, 
//     &right_msg, 
//     &right_subscription_callback, 
//     ON_NEW_DATA));



// leftMotors = addPinsToLeftMotorSet();
// rightMotors = addPinsToRightMotorSet();

// }

// void loop() {
//   //delay(100);
//   RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

//    dataHandling(motor_command_data, leftMotors, rightMotors);
//   // digitalWrite(RIGHT_LED_PIN, HIGH);
//   // delay(100);
//   // digitalWrite(RIGHT_LED_PIN, LOW);
//   // delay(100);


// }





// #include <Arduino.h>
// #include <micro_ros_platformio.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/float32.h>

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

// rcl_subscription_t subscriber;
// std_msgs__msg__Float32 msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;

// #define LED_PIN 25

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// // Error handle loop
// void error_loop() {
//   // while(1) {
//   //   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//   //   delay(100);
//   // }
// }

// void subscription_callback(const void * msgin)
// {  
//   const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;

//   if (msg->data > 0)
//   {
//     digitalWrite(LED_PIN, HIGH);
//   }
//   else
//   {
//     digitalWrite(LED_PIN, LOW);
//   }
// }

// void setup() {
//   // Configure serial transport
//   Serial.begin(115200);
//   set_microros_serial_transports(Serial);

//   pinMode(LED_PIN, OUTPUT);
//   // digitalWrite(LED_PIN, HIGH); 

//   delay(2000);

//   allocator = rcl_get_default_allocator();

//   //create init_options
//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//   // create node
//   RCCHECK(rclc_node_init_default(&node, "LEDreceiver", "", &support));

//   // create subscriber
//   RCCHECK(rclc_subscription_init_default(
//     &subscriber,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
//     "motor_command/left_trigger"));

//   // create executor
//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//   RCCHECK(rclc_executor_add_subscription(
//     &executor, 
//     &subscriber, 
//     &msg, 
//     &subscription_callback, 
//     ON_NEW_DATA));

// }

// void loop() {
//   //delay(100);
//   RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  
  

// }
