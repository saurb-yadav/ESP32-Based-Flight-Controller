#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <MPU6050_tockn.h>

// WiFi SoftAP credentials
const char* ssid = "Drone_Controller";
const char* password = "drone12345";
const int udpPort = 4210;

#define MPU_SDA 21
#define MPU_SCL 22
#define MOTOR1_PIN 13  // Front right motor
#define MOTOR2_PIN 25  // Rear right motor
#define MOTOR3_PIN 23  // Rear left motor
#define MOTOR4_PIN 27  // Front left motor
#define LED_PIN 2

// MPU6050 variables
MPU6050 mpu6050(Wire);
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accOffsetX = 0, accOffsetY = 0, accOffsetZ = 0;
float roll = 0, pitch = 0, yaw = 0;
bool calibrated = false;
int calibrationSamples = 0;
const int CALIBRATION_SAMPLES = 30;

// Control setpoints
float rollSetpoint = 0, pitchSetpoint = 0, yawSetpoint = 0;

// PID variables
float complementaryAngleRoll = 0, complementaryAnglePitch = 0;
float DesiredAngleRoll = 0, DesiredAnglePitch = 0, DesiredRateYaw = 0;
float RateRoll = 0, RatePitch = 0, RateYaw = 0;
float ErrorAngleRoll = 0, ErrorAnglePitch = 0, ErrorRateRoll = 0, ErrorRatePitch = 0, ErrorRateYaw = 0;
float PrevErrorAngleRoll = 0, PrevErrorAnglePitch = 0, PrevErrorRateRoll = 0, PrevErrorRatePitch = 0, PrevErrorRateYaw = 0;
float PrevItermAngleRoll = 0, PrevItermAnglePitch = 0, PrevItermRateRoll = 0, PrevItermRatePitch = 0, PrevItermRateYaw = 0;
float PtermRoll = 0, ItermRoll = 0, DtermRoll = 0, PIDOutputRoll = 0;
float PtermPitch = 0, ItermPitch = 0, DtermPitch = 0, PIDOutputPitch = 0;
float PtermYaw = 0, ItermYaw = 0, DtermYaw = 0, PIDOutputYaw = 0;
float InputRoll = 0, InputPitch = 0, InputYaw = 0;
float DesiredRateRoll = 0, DesiredRatePitch = 0;

// PID tuning parameters
const float PAngleRoll = 2.0, IAngleRoll = 0.5, DAngleRoll = 0.007;
const float PAnglePitch = 2.0, IAnglePitch = 0.5, DAnglePitch = 0.007;
const float PRateRoll = 0.625, IRateRoll = 2.1, DRateRoll = 0.0088;
const float PRatePitch = 0.625, IRatePitch = 2.1, DRatePitch = 0.0088;
const float PRateYaw = 4.0, IRateYaw = 3.0, DRateYaw = 0.0;

// Motor control
Servo motor1, motor2, motor3, motor4;
int baseThrottle = 1000;  // Default throttle (min: 1000, max: 2000)
const int MIN_MOTOR_SPEED = 1000;
const int MAX_MOTOR_SPEED = 2000;
const int ARM_CONFIRMATION_SPEED = 1100;  // Speed for arm confirmation
bool motorsArmed = false;

// Communication
WiFiUDP udp;
char packetBuffer[255];

// Loop timing
unsigned long currentTime = 0;
unsigned long previousTime = 0;
float elapsedTime = 0;
const float LOOP_FREQUENCY = 250;  // Hz
const float LOOP_PERIOD = 1000000 / LOOP_FREQUENCY;  // in microseconds

void setup() {
  Serial.begin(115200);
  
  // Setup LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize I2C
  Wire.begin(MPU_SDA, MPU_SCL);
  delay(100);  // Ensure I2C bus stabilizes
  
  // Search and set MPU6050 I2C port
  Serial.println("Searching for MPU6050...");
  if (!mpu6050.searchAndSetI2CPort(Wire, true)) {
    Serial.println("MPU6050 not found. Check connections and restart.");
    while (1);  // Halt execution
  }
  
  // Initialize MPU6050
  mpu6050.begin();
  Serial.println("Calibrating MPU6050...");
  delay(1000);  // Allow MPU to stabilize
  
  // Setup WiFi SoftAP
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Begin UDP
  udp.begin(udpPort);
  Serial.print("UDP server started on port ");
  Serial.println(udpPort);
  
  // Initialize ESCs
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  motor1.setPeriodHertz(50);  // Standard 50Hz PWM for ESCs
  motor2.setPeriodHertz(50);
  motor3.setPeriodHertz(50);
  motor4.setPeriodHertz(50);
  
  // Attach motors to pins
  motor1.attach(MOTOR1_PIN, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  motor2.attach(MOTOR2_PIN, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  motor3.attach(MOTOR3_PIN, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  motor4.attach(MOTOR4_PIN, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  
  // Initialize motors (arm ESCs)
  motor1.writeMicroseconds(MIN_MOTOR_SPEED);
  motor2.writeMicroseconds(MIN_MOTOR_SPEED);
  motor3.writeMicroseconds(MIN_MOTOR_SPEED);
  motor4.writeMicroseconds(MIN_MOTOR_SPEED);
  delay(2000);  // Wait for ESCs to initialize
  
  Serial.println("Setup complete!");
}

void calibrateMPU() {
  // Collect samples for calibration
  if (calibrationSamples < CALIBRATION_SAMPLES) {
    gyroOffsetX += mpu6050.getGyroX();
    gyroOffsetY += mpu6050.getGyroY();
    gyroOffsetZ += mpu6050.getGyroZ();
    
    accOffsetX += mpu6050.getAccX();
    accOffsetY += mpu6050.getAccY();
    accOffsetZ += mpu6050.getAccZ() - 1.0;  // Subtract gravity
    
    calibrationSamples++;
    
    if (calibrationSamples == CALIBRATION_SAMPLES) {
      // Calculate average offsets
      gyroOffsetX /= CALIBRATION_SAMPLES;
      gyroOffsetY /= CALIBRATION_SAMPLES;
      gyroOffsetZ /= CALIBRATION_SAMPLES;
      
      accOffsetX /= CALIBRATION_SAMPLES;
      accOffsetY /= CALIBRATION_SAMPLES;
      accOffsetZ /= CALIBRATION_SAMPLES;
      
      calibrated = true;
      digitalWrite(LED_PIN, HIGH);  // LED on when calibrated
      Serial.println("MPU6050 calibration complete!");
      Serial.println("Gyro offsets: " + String(gyroOffsetX) + ", " + String(gyroOffsetY) + ", " + String(gyroOffsetZ));
      Serial.println("Acc offsets: " + String(accOffsetX) + ", " + String(accOffsetY) + ", " + String(accOffsetZ));
    }
  }
}

void readMPU() {
  mpu6050.update();
  
  if (calibrated) {
    // Apply offsets
    float gyroX = mpu6050.getGyroX() - gyroOffsetX;
    float gyroY = mpu6050.getGyroY() - gyroOffsetY;
    float gyroZ = mpu6050.getGyroZ() - gyroOffsetZ;
    
    float accX = mpu6050.getAccX() - accOffsetX;
    float accY = mpu6050.getAccY() - accOffsetY;
    float accZ = mpu6050.getAccZ() - accOffsetZ;
    
    // Use complementary filter for attitude estimation
    // Get angles from accelerometer
    float accelRoll = atan2(accY, accZ) * 180.0 / PI;
    float accelPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
    
    // Integrate gyro rates
    roll = 0.98 * (roll + gyroX * elapsedTime) + 0.02 * accelRoll;
    pitch = 0.98 * (pitch + gyroY * elapsedTime) + 0.02 * accelPitch;
    yaw += gyroZ * elapsedTime; // Simple integration for yaw
    
    // Update PID variables
    complementaryAngleRoll = roll;
    complementaryAnglePitch = pitch;
    RateRoll = gyroX;
    RatePitch = gyroY;
    RateYaw = gyroZ;
  }
}

// PID equation for angle and rate control
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm, float& Pterm, float& Iterm, float& Dterm, float& PIDOutput) {
  Pterm = P * Error;
  Iterm = PrevIterm + (I * (Error + PrevError) * (elapsedTime / 2));
  Iterm = (Iterm > 400) ? 400 : ((Iterm < -400) ? -400 : Iterm);
  Dterm = D * ((Error - PrevError) / elapsedTime);
  PIDOutput = Pterm + Iterm + Dterm;
  PIDOutput = (PIDOutput > 400) ? 400 : ((PIDOutput < -400) ? -400 : PIDOutput);
}

void computeControl() {
  // Roll angle PID
  ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
  pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll, PrevItermAngleRoll, PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // Pitch angle PID
  ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
  pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch, PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // Roll rate PID
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll, PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch rate PID
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch, PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw rate PID
  ErrorRateYaw = DesiredRateYaw - RateYaw;
  pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw, PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;
}

void setMotorSpeeds() {
  if (!motorsArmed || baseThrottle < 1050) {
    // Motors off if not armed or throttle too low
    motor1.writeMicroseconds(MIN_MOTOR_SPEED);
    motor2.writeMicroseconds(MIN_MOTOR_SPEED);
    motor3.writeMicroseconds(MIN_MOTOR_SPEED);
    motor4.writeMicroseconds(MIN_MOTOR_SPEED);
    return;
  }
  
  // Motor mixing with PID outputs
  int motor1Speed = baseThrottle - InputRoll + InputPitch - InputYaw; // Front right
  int motor2Speed = baseThrottle - InputRoll - InputPitch + InputYaw; // Rear right
  int motor3Speed = baseThrottle + InputRoll - InputPitch - InputYaw; // Rear left
  int motor4Speed = baseThrottle + InputRoll + InputPitch + InputYaw; // Front left
  
  // Constrain motor speeds
  motor1Speed = constrain(motor1Speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  motor2Speed = constrain(motor2Speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  motor3Speed = constrain(motor3Speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  motor4Speed = constrain(motor4Speed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  
  // Write to ESCs
  motor1.writeMicroseconds(motor1Speed);
  motor2.writeMicroseconds(motor2Speed);
  motor3.writeMicroseconds(motor3Speed);
  motor4.writeMicroseconds(motor4Speed);
}

void armMotors() {
  motorsArmed = true;
  digitalWrite(LED_PIN, HIGH);
  
  // Arm confirmation - briefly spin motors at low speed
  motor1.writeMicroseconds(ARM_CONFIRMATION_SPEED);
  motor2.writeMicroseconds(ARM_CONFIRMATION_SPEED);
  motor3.writeMicroseconds(ARM_CONFIRMATION_SPEED);
  motor4.writeMicroseconds(ARM_CONFIRMATION_SPEED);
  delay(300);  // Spin for 300ms
  motor1.writeMicroseconds(MIN_MOTOR_SPEED);
  motor2.writeMicroseconds(MIN_MOTOR_SPEED);
  motor3.writeMicroseconds(MIN_MOTOR_SPEED);
  motor4.writeMicroseconds(MIN_MOTOR_SPEED);
  
  Serial.println("Motors armed with confirmation spin");
}

void disarmMotors() {
  motorsArmed = false;
  digitalWrite(LED_PIN, LOW);
  
  motor1.writeMicroseconds(MIN_MOTOR_SPEED);
  motor2.writeMicroseconds(MIN_MOTOR_SPEED);
  motor3.writeMicroseconds(MIN_MOTOR_SPEED);
  motor4.writeMicroseconds(MIN_MOTOR_SPEED);
  
  Serial.println("Motors disarmed");
}

void processUDPData() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;  // Null terminate
      
      String message = String(packetBuffer);
      Serial.println("UDP: " + message);
      
      // Parse standard message format: (Throttle,Pitch,Roll,Yaw)
      // Or special commands: ARM, DISARM
      if (message.startsWith("(") && message.endsWith(")")) {
        // It's a control message in format (Throttle,Pitch,Roll,Yaw)
        message = message.substring(1, message.length() - 1); // Remove parentheses
        
        // Split by commas
        int firstComma = message.indexOf(',');
        int secondComma = message.indexOf(',', firstComma + 1);
        int thirdComma = message.indexOf(',', secondComma + 1);
        
        if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
          String throttleStr = message.substring(0, firstComma);
          String pitchStr = message.substring(firstComma + 1, secondComma);
          String rollStr = message.substring(secondComma + 1, thirdComma);
          String yawStr = message.substring(thirdComma + 1);
          
          // Convert to values
          int throttleVal = throttleStr.toInt();
          float pitchVal = pitchStr.toFloat();
          float rollVal = rollStr.toFloat();
          float yawVal = yawStr.toFloat();
          
          // Apply controls
          baseThrottle = 1000 + constrain(throttleVal, 0, 1000);
          DesiredAnglePitch = constrain(pitchVal, -30, 30);
          DesiredAngleRoll = constrain(rollVal, -30, 30);
          DesiredRateYaw = constrain(yawVal, -180, 180);
          
          // Debug output to Serial
          Serial.println("Controls: T=" + String(baseThrottle) + 
                        " P=" + String(DesiredAnglePitch) + 
                        " R=" + String(DesiredAngleRoll) + 
                        " Y=" + String(DesiredRateYaw));
        }
      }
      else if (message == "ARM") {
        if (!motorsArmed) {
          armMotors();
        }
      }
      else if (message == "DISARM") {
        if (motorsArmed) {
          disarmMotors();
        }
      }
      else if (message.indexOf(':') != -1) {
        // Process as CMD:VAL format for non-PID commands
        int colonIndex = message.indexOf(':');
        String command = message.substring(0, colonIndex);
        String value = message.substring(colonIndex + 1);
        float floatValue = value.toFloat();
        
        if (command == "RESET_YAW" && floatValue == 1) {
          yaw = 0;
          DesiredRateYaw = 0;
          Serial.println("Reset YAW to zero");
        }
      }
    }
  }
}

void loop() {
  // Track loop timing
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000.0; // Convert to seconds
  
  // Verify MPU6050 I2C address periodically (every 5 seconds)
  static unsigned long lastAddressCheck = 0;
  if (millis() - lastAddressCheck > 5000) {
    if (!mpu6050.verifyI2CAddress(true)) {
      Serial.println("MPU6050 address verification failed. Disarming motors and halting.");
      disarmMotors();
      while (1); // Halt execution
    }
    lastAddressCheck = millis();
  }
  
  // Read MPU data
  if (!calibrated) {
    mpu6050.update();
    calibrateMPU();
  } else {
    readMPU();
    computeControl(); // Compute PID controls
  }
  
  // Process incoming UDP data
  processUDPData();
  
  // Set motor speeds
  setMotorSpeeds();
  
  // Print debug data occasionally
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 500 && motorsArmed) {  // Print every 500ms when armed
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("\tPitch: ");
    Serial.print(pitch);
    Serial.print("\tYaw: ");
    Serial.println(yaw);
    lastDebugTime = millis();
  }
  
  // Maintain consistent loop frequency
  while (micros() - currentTime < LOOP_PERIOD) {
    // Wait
  }
  previousTime = currentTime;
}