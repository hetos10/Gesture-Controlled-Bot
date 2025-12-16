#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// L298N Motor Driver Pins (6-pin configuration)
// Motor A (Left Motors)
#define MOTOR_A_IN1 16  // IN1 - Left motor direction control
#define MOTOR_A_IN2 17  // IN2 - Left motor direction control
#define MOTOR_A_ENA 23  // ENA - Left motor speed (PWM)

// Motor B (Right Motors)
#define MOTOR_B_IN3 4   // IN3 - Right motor direction control
#define MOTOR_B_IN4 5   // IN4 - Right motor direction control
#define MOTOR_B_ENB 22  // ENB - Right motor speed (PWM)

// PWM Configuration
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

// Micro-ROS variables
rcl_subscription_t subscription;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Error handling macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Function prototypes
void setup_micro_ros();
void subscription_callback(const void * msgin);
void control_motors(float linear_x, float angular_z);
void set_motor_speed(int motor_pin_in1, int motor_pin_in2, int motor_pwm_pin, int pwm_channel, float speed);
void error_loop();

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n=== ESP32 Micro-ROS Motor Controller (Serial USB) ===");
  Serial.println("Initializing...");
  
  // Setup PWM for motor control
  ledcSetup(PWM_CHANNEL_A, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A_ENA, PWM_CHANNEL_A);
  ledcAttachPin(MOTOR_B_ENB, PWM_CHANNEL_B);
  
  // Setup motor control pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT);
  pinMode(MOTOR_B_IN4, OUTPUT);
  
  // Stop motors initially
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
  
  Serial.println("Motor pins configured.");
  
  // Setup Micro-ROS with Serial USB
  setup_micro_ros();
  
  Serial.println("Setup complete! Waiting for messages...");
}

void loop() {
  delay(100);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

void setup_micro_ros() {
  Serial.println("Setting up Micro-ROS...");
  
  // Set up Serial transport
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  
  // Create support object
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("  - Support initialized");
  
  // Create node
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_controller", "", &support));
  Serial.println("  - Node created");
  
  // Create subscription
  RCCHECK(rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/wheel_controller/cmd_vel_unstamped"));
  Serial.println("  - Subscription to /wheel_controller/cmd_vel_unstamped created");
  
  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscription, &msg, &subscription_callback, ON_NEW_DATA));
  Serial.println("  - Executor initialized");
  
  Serial.println("Micro-ROS initialized successfully!");
  Serial.println("Waiting for agent connection...");
}

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;
  
  Serial.println("================================================");
  Serial.println("MESSAGE RECEIVED FROM ROS!");
  Serial.print("  Linear X:  ");
  Serial.println(linear_x);
  Serial.print("  Angular Z: ");
  Serial.println(angular_z);
  Serial.println("================================================");
  
  control_motors(linear_x, angular_z);
}

void control_motors(float linear_x, float angular_z) {
  // Convert Twist message to motor speeds
  // linear_x: forward/backward motion
  // angular_z: rotation
  
  // Simple differential drive kinematics
  float wheel_separation = 0.2;  // Adjust based on your robot's wheel separation (meters)
  float max_speed = 1.0;         // Maximum linear speed (m/s)
  
  float left_speed = linear_x - (angular_z * wheel_separation / 2.0);
  float right_speed = linear_x + (angular_z * wheel_separation / 2.0);
  
  // Normalize speeds to [-1, 1] range
  float speed_factor = max(abs(left_speed), abs(right_speed)) / max_speed;
  if (speed_factor > 1.0) {
    left_speed /= speed_factor;
    right_speed /= speed_factor;
  }
  
  Serial.print("  Calculated Left Speed:  ");
  Serial.println(left_speed);
  Serial.print("  Calculated Right Speed: ");
  Serial.println(right_speed);
  
  // Set motor speeds
  set_motor_speed(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_ENA, PWM_CHANNEL_A, left_speed);
  set_motor_speed(MOTOR_B_IN3, MOTOR_B_IN4, MOTOR_B_ENB, PWM_CHANNEL_B, right_speed);
}

void set_motor_speed(int motor_pin_in1, int motor_pin_in2, int motor_pwm_pin, int pwm_channel, float speed) {
  // speed: -1.0 (full reverse) to 1.0 (full forward)
  
  int pwm_value = (int)(abs(speed) * 255);
  
  Serial.print("    Setting PWM value: ");
  Serial.println(pwm_value);
  
  if (speed > 0) {
    // Forward
    digitalWrite(motor_pin_in1, HIGH);
    digitalWrite(motor_pin_in2, LOW);
    Serial.println("    Direction: FORWARD");
  } else if (speed < 0) {
    // Backward
    digitalWrite(motor_pin_in1, LOW);
    digitalWrite(motor_pin_in2, HIGH);
    Serial.println("    Direction: BACKWARD");
  } else {
    // Stop
    digitalWrite(motor_pin_in1, LOW);
    digitalWrite(motor_pin_in2, LOW);
    Serial.println("    Direction: STOP");
  }
  
  ledcWrite(pwm_channel, pwm_value);
}

void error_loop() {
  Serial.println("\n\n!!! ERROR !!!");
  Serial.println("Restarting in 5 seconds...");
  delay(5000);
  ESP.restart();
}