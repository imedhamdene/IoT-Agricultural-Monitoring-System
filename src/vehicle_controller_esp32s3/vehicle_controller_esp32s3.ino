// ESP32-S3 Master Code (Arduino IDE)

#include <HardwareSerial.h> // For UART communication with ESP32-CAM
#include <ESP32Servo.h>     // For the MG90S servo

// --- Pin Definitions for ESP32-S3 ---
// ESP-CAM Communication (UART2)
#define CAM_SERIAL_RX_PIN 18 // ESP32-S3 RX to ESP32-CAM TX
#define CAM_SERIAL_TX_PIN 17 // ESP32-S3 TX to ESP32-CAM RX
HardwareSerial CamSerial(2); // Use UART2

// Servo Motor
#define SERVO_PIN 10
Servo cameraServo;

// L298N Motor Control
// Motor 1 (Left Track)
#define IN1_PIN 1
#define IN2_PIN 2
#define ENA_PIN 3 // PWM for speed control
// Motor 2 (Right Track)
#define IN3_PIN 4
#define IN4_PIN 5
#define ENB_PIN 6 // PWM for speed control

// HC-SR04 Ultrasonic Sensors (6x)
// Sensor 1 (Front-Left)
#define S1_TRIG_PIN 7
#define S1_ECHO_PIN 8
// Sensor 2 (Front-Right)
#define S2_TRIG_PIN 9
#define S2_ECHO_PIN 11
// Sensor 3 (Left-Side)
#define S3_TRIG_PIN 12
#define S3_ECHO_PIN 13
// Sensor 4 (Right-Side)
#define S4_TRIG_PIN 14
#define S4_ECHO_PIN 15
// Sensor 5 (Rear-Left)
#define S5_TRIG_PIN 16
#define S5_ECHO_PIN 19
// Sensor 6 (Rear-Right)
#define S6_TRIG_PIN 20
#define S6_ECHO_PIN 21

// PWM Settings for DC Motors
const int freq = 30000; // PWM frequency
const int resolution = 8; // 8-bit resolution (0-255)
const int motor1Channel = 0;
const int motor2Channel = 1;

// --- Sensor Variables ---
long duration;
int distanceCm[6]; // To store distances for all 6 sensors

// --- Function Prototypes ---
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();
long readUltrasonicSensor(int trigPin, int echoPin);
void sendCaptureCommand();
void rotateCamera(int angle);
void checkObstaclesAndMove();


void setup() {
  Serial.begin(115200); // For debugging output to PC
  CamSerial.begin(115200, SERIAL_8N1, CAM_SERIAL_RX_PIN, CAM_SERIAL_TX_PIN); // For communication with ESP32-CAM
  Serial.println("ESP32-S3 Master Started");

  // --- Servo Setup ---
  ESP32PWM::allocateTimer(0);
  cameraServo.setPeriodHertz(50);    // standard 50 hz servo
  cameraServo.attach(SERVO_PIN, 500, 2400); // Attach servo to pin, adjust min/max pulse width if needed
  rotateCamera(90); // Center the camera initially

  // --- Motor Driver Setup (L298N) ---
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // Configure PWM for motor speed control
  ledcSetup(motor1Channel, freq, resolution);
  ledcAttachPin(ENA_PIN, motor1Channel);
  ledcSetup(motor2Channel, freq, resolution);
  ledcAttachPin(ENB_PIN, motor2Channel);
  stopMotors(); // Ensure motors are stopped at start

  // --- HC-SR04 Setup ---
  pinMode(S1_TRIG_PIN, OUTPUT);
  pinMode(S1_ECHO_PIN, INPUT);
  pinMode(S2_TRIG_PIN, OUTPUT);
  pinMode(S2_ECHO_PIN, INPUT);
  pinMode(S3_TRIG_PIN, OUTPUT);
  pinMode(S3_ECHO_PIN, INPUT);
  pinMode(S4_TRIG_PIN, OUTPUT);
  pinMode(S4_ECHO_PIN, INPUT);
  pinMode(S5_TRIG_PIN, OUTPUT);
  pinMode(S5_ECHO_PIN, INPUT);
  pinMode(S6_TRIG_PIN, OUTPUT);
  pinMode(S6_ECHO_PIN, INPUT);

  delay(2000); // Give components time to initialize
  Serial.println("Initialization Complete.");
}

void loop() {
  // Read all ultrasonic sensors
  distanceCm[0] = readUltrasonicSensor(S1_TRIG_PIN, S1_ECHO_PIN); // Front-Left
  distanceCm[1] = readUltrasonicSensor(S2_TRIG_PIN, S2_ECHO_PIN); // Front-Right
  distanceCm[2] = readUltrasonicSensor(S3_TRIG_PIN, S3_ECHO_PIN); // Left-Side
  distanceCm[3] = readUltrasonicSensor(S4_TRIG_PIN, S4_ECHO_PIN); // Right-Side
  distanceCm[4] = readUltrasonicSensor(S5_TRIG_PIN, S5_ECHO_PIN); // Rear-Left
  distanceCm[5] = readUltrasonicSensor(S6_TRIG_PIN, S6_ECHO_PIN); // Rear-Right

  Serial.print("Distances (FL, FR, L, R, RL, RR): ");
  for(int i=0; i<6; i++) {
    Serial.print(distanceCm[i]);
    Serial.print("cm ");
  }
  Serial.println();

  // Basic Autonomy Logic (Placeholder - you'll expand this)
  checkObstaclesAndMove();

  // Example: Take a picture every 10 seconds and rotate camera
  static unsigned long lastPhotoTime = 0;
  if (millis() - lastPhotoTime > 10000) { // Take a pic every 10 seconds
    Serial.println("Time to take a picture!");
    sendCaptureCommand();
    // Rotate camera for different views
    rotateCamera(45);  // Left
    delay(2000);
    sendCaptureCommand();
    rotateCamera(135); // Right
    delay(2000);
    sendCaptureCommand();
    rotateCamera(90);  // Center
    lastPhotoTime = millis();
  }

  // Check for response from ESP32-CAM
  while (CamSerial.available()) {
    String response = CamSerial.readStringUntil('\n');
    Serial.print("CAM Response: ");
    Serial.println(response);
  }

  delay(100); // Small delay for loop iteration
}

// --- Movement Functions ---
void moveForward(int speed) {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  ledcWrite(motor1Channel, speed);
  ledcWrite(motor2Channel, speed);
  Serial.println("Moving Forward");
}

void moveBackward(int speed) {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  ledcWrite(motor1Channel, speed);
  ledcWrite(motor2Channel, speed);
  Serial.println("Moving Backward");
}

void turnLeft(int speed) {
  digitalWrite(IN1_PIN, LOW);  // Left motor backward
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH); // Right motor forward
  digitalWrite(IN4_PIN, LOW);
  ledcWrite(motor1Channel, speed);
  ledcWrite(motor2Channel, speed);
  Serial.println("Turning Left");
}

void turnRight(int speed) {
  digitalWrite(IN1_PIN, HIGH); // Left motor forward
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);  // Right motor backward
  digitalWrite(IN4_PIN, HIGH);
  ledcWrite(motor1Channel, speed);
  ledcWrite(motor2Channel, speed);
  Serial.println("Turning Right");
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  ledcWrite(motor1Channel, 0);
  ledcWrite(motor2Channel, 0);
  Serial.println("Motors Stopped");
}

// --- Ultrasonic Sensor Function ---
long readUltrasonicSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 50000); // Timeout after 50ms (for ~8.5m distance)
  if (duration == 0) return 999; // Sensor timed out, assume far away
  return duration * 0.034 / 2; // Speed of sound is 0.034 cm/microsecond
}

// --- ESP32-CAM Communication ---
void sendCaptureCommand() {
  CamSerial.println("CAPTURE_PHOTO"); // Send a specific command string
  Serial.println("Sent 'CAPTURE_PHOTO' command to ESP32-CAM");
}

// --- Servo Control ---
void rotateCamera(int angle) {
  cameraServo.write(angle);
  Serial.print("Rotating camera to: ");
  Serial.println(angle);
  delay(100); // Give servo time to move
}

// --- Basic Autonomy Logic ---
void checkObstaclesAndMove() {
  int minFrontDistance = min(distanceCm[0], distanceCm[1]); // Front-Left & Front-Right

  if (minFrontDistance < 30) { // If obstacle is closer than 30cm in front
    stopMotors();
    Serial.println("Obstacle detected in front! Backing up...");
    moveBackward(150);
    delay(1000); // Back up for 1 second
    stopMotors();
    // Decide to turn left or right based on side sensors
    if (distanceCm[2] > distanceCm[3] && distanceCm[2] > 30) { // If left side is clearer
      Serial.println("Turning Left to avoid obstacle.");
      turnLeft(150);
      delay(1000);
    } else if (distanceCm[3] > distanceCm[2] && distanceCm[3] > 30) { // If right side is clearer
      Serial.println("Turning Right to avoid obstacle.");
      turnRight(150);
      delay(1000);
    } else { // Both sides blocked or equally clear, try a random turn or just stop
      Serial.println("Both sides seem blocked or unclear. Stopping.");
      stopMotors();
      // You might add more sophisticated logic here like waiting or random turns
    }
  } else {
    moveForward(100); // Keep moving forward at a moderate speed
  }
}