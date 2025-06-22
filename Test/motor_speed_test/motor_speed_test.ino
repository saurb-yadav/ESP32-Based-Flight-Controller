#include <Arduino.h>
#include <ESP32Servo.h>

// Motor pin definitions
#define MOTOR1_PIN 13  // Front right motor
#define MOTOR2_PIN 25  // Rear right motor
#define MOTOR3_PIN 23  // Rear left motor
#define MOTOR4_PIN 27  // Front left motor
#define LED_PIN 2      // Onboard LED for status

// Motor control constants
const int MIN_MOTOR_SPEED = 1000;  // Minimum ESC PWM (microseconds)
const int MAX_MOTOR_SPEED = 2000;  // Maximum ESC PWM (microseconds)
const int TEST_SPEED = 1100;       // Test speed (microseconds)
const int TEST_DURATION = 5000;    // Test duration per motor (milliseconds)

// Servo objects for motor control
Servo motor1, motor2, motor3, motor4;

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  
  // Setup LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Allocate timers for PWM
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Configure Servo objects
  motor1.setPeriodHertz(50);  // Standard 50Hz PWM for ESCs
  motor2.setPeriodHertz(50);
  motor3.setPeriodHertz(50);
  motor4.setPeriodHertz(50);
  
  // Attach motors to pins
  motor1.attach(MOTOR1_PIN, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  motor2.attach(MOTOR2_PIN, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  motor3.attach(MOTOR3_PIN, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  motor4.attach(MOTOR4_PIN, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  
  // Initialize ESCs (set to minimum speed)
  Serial.println("Initializing ESCs...");
  motor1.writeMicroseconds(MIN_MOTOR_SPEED);
  motor2.writeMicroseconds(MIN_MOTOR_SPEED);
  motor3.writeMicroseconds(MIN_MOTOR_SPEED);
  motor4.writeMicroseconds(MIN_MOTOR_SPEED);
  delay(2000);  // Wait for ESCs to initialize
  
  Serial.println("ESC initialization complete. Starting motor test in 5 seconds...");
  delay(5000);  // Initial delay before starting tests
}

void loop() {
  // Test Motor 1 (Front right)
  Serial.println("Testing Motor 1 (Front right)");
  digitalWrite(LED_PIN, HIGH);  // LED on during test
  motor1.writeMicroseconds(TEST_SPEED);
  delay(TEST_DURATION);
  motor1.writeMicroseconds(MIN_MOTOR_SPEED);
  digitalWrite(LED_PIN, LOW);   // LED off after test
  Serial.println("Motor 1 test complete");
  delay(1000);  // Short pause between tests
  
  // Test Motor 2 (Rear right)
  Serial.println("Testing Motor 2 (Rear right)");
  digitalWrite(LED_PIN, HIGH);
  motor2.writeMicroseconds(TEST_SPEED);
  delay(TEST_DURATION);
  motor2.writeMicroseconds(MIN_MOTOR_SPEED);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Motor 2 test complete");
  delay(1000);
  
  // Test Motor 3 (Rear left)
  Serial.println("Testing Motor 3 (Rear left)");
  digitalWrite(LED_PIN, HIGH);
  motor3.writeMicroseconds(TEST_SPEED);
  delay(TEST_DURATION);
  motor3.writeMicroseconds(MIN_MOTOR_SPEED);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Motor 3 test complete");
  delay(1000);
  
  // Test Motor 4 (Front left)
  Serial.println("Testing Motor 4 (Front left)");
  digitalWrite(LED_PIN, HIGH);
  motor4.writeMicroseconds(TEST_SPEED);
  delay(TEST_DURATION);
  motor4.writeMicroseconds(MIN_MOTOR_SPEED);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Motor 4 test complete");
  
  // Stop further execution after one complete test cycle
  Serial.println("All motor tests complete. Halting.");
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink LED to indicate halt
    delay(500);
  }
}
