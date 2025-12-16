#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <WiFi.h>

// WiFi & micro-ROS Agent Settings
char ssid[] = "Gratitude4G";
char password[] = "Qzpm#2204";
char agent_ip[] = "192.168.29.236";
uint32_t agent_port = 8888;

// L298N Motor Driver Pins (6-pin configuration)
#define MOTOR_A_IN1 16  // IN1 - Left motor direction control
#define MOTOR_A_IN2 17  // IN2 - Left motor direction control
#define MOTOR_A_ENA 23  // ENA - Left motor speed (PWM)

#define MOTOR_B_IN3 4   // IN3 - Right motor direction control
#define MOTOR_B_IN4 5   // IN4 - Right motor direction control
#define MOTOR_B_ENB 22  // ENB - Right motor speed (PWM)

// PWM Configuration
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

// Micro-ROS objects
rcl_subscription_t subscription;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Safety timeout for motor commands
unsigned long last_message_time = 0;
const unsigned long MESSAGE_TIMEOUT = 1000;  // 1 second timeout

// Error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void subscription_callback(const void * msgin);
void control_motors(float linear_x, float angular_z);
void set_motor_speed(int in1, int in2, int pwm_pin, int pwm_channel, float speed);
void setup_wifi();
void setup_micro_ros();
void error_loop();

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== ESP32 Micro-ROS Motor Controller (WiFi UDP) ===");
  Serial.println("Booting...");

  // Initialize Twist message to zero (CRITICAL!)
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

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

  // Motors OFF initially
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);
  digitalWrite(MOTOR_B_IN4, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);

  Serial.println("Motor pins configured.");

  // Connect WiFi
  setup_wifi();

  // Setup micro-ROS (UDP transport + node + subscription)
  setup_micro_ros();

  Serial.println("Setup complete! Waiting for Twist messages...");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
  
  // Safety: Stop motors if no message received for 1 second
  if (millis() - last_message_time > MESSAGE_TIMEOUT) {
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    control_motors(0.0, 0.0);
  }
}

void setup_wifi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected!");
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n✗ Failed to connect to WiFi");
    error_loop();
  }
}

void setup_micro_ros() {
  allocator = rcl_get_default_allocator();

  // CRITICAL: Set WiFi transport with UDP
  Serial.println("Setting up Micro-ROS WiFi transport...");
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  Serial.println("  ✓ WiFi transport initialized");

  // Initialize support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("  ✓ Support initialized");

  // Create node
  RCCHECK(rclc_node_init_default(
    &node,
    "esp32_motor_controller",
    "",
    &support));
  Serial.println("  ✓ Node created");

  // Create subscription
  RCCHECK(rclc_subscription_init_default(
    &subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/wheel_controller/cmd_vel_unstamped"));
  Serial.println("  ✓ Subscription created");

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscription,
    &msg,
    &subscription_callback,
    ON_NEW_DATA));
  Serial.println("  ✓ Executor initialized");

  Serial.println("Micro-ROS initialized successfully!");
  Serial.println("Waiting for agent connection and Twist messages...");
}

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;

  // Update last message time (for safety timeout)
  last_message_time = millis();

  Serial.println("================================================");
  Serial.println("✓ MESSAGE RECEIVED FROM ROS!");
  Serial.print("  Linear X:  ");
  Serial.println(linear_x);
  Serial.print("  Angular Z: ");
  Serial.println(angular_z);
  Serial.println("================================================");

  control_motors(linear_x, angular_z);
}

void control_motors(float linear_x, float angular_z) {
  // Differential drive kinematics
  float wheel_separation = 0.10;  // Adjust based on your robot (meters)

  float left_speed = linear_x - (angular_z * wheel_separation / 2.0);
  float right_speed = linear_x + (angular_z * wheel_separation / 2.0);

  // Normalize speeds
  float max_speed = max(abs(left_speed), abs(right_speed));
  if (max_speed > 1.0) {
    left_speed /= max_speed;
    right_speed /= max_speed;
  }

  Serial.print("  Left Speed:  ");
  Serial.print(left_speed);
  Serial.print("  | Right Speed: ");
  Serial.println(right_speed);

  set_motor_speed(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_ENA, PWM_CHANNEL_A, left_speed);
  set_motor_speed(MOTOR_B_IN3, MOTOR_B_IN4, MOTOR_B_ENB, PWM_CHANNEL_B, right_speed);
}

void set_motor_speed(int in1, int in2, int pwm_pin, int pwm_channel, float speed) {
  int pwm_value = (int)(abs(speed) * 255);  // Full PWM range 0-255

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  ledcWrite(pwm_channel, pwm_value);
}

void error_loop() {
  while (1) {
    Serial.println("ERROR! Restarting...");
    delay(3000);
    ESP.restart();
  }
}