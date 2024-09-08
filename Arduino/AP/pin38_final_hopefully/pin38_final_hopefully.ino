#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <driver/ledc.h>  // Explicitly include the LED control library

// ESP32------------------------------------------------------
// MAC address of the receiver ESP32 - pin30
uint8_t espBoardA_address[] = { 0xA0, 0xA3, 0xB3, 0x2A, 0xDE, 0x3C };  // Replace with actual MAC address of the receiver ESP32

// Structure to hold data to be sent
typedef struct struct_message {
  int angleX, angleY, angleZ;  // Gyroscope data
  bool gyroscopeDataSent;      // Flag to indicate if gyroscope data was sent
  bool gyroscopeEnterPressed;
  bool BIsPressed;
  bool outputCheckCalled;
  bool ultrasonicObjectDetected;
  bool waterFilled;  // water part completion condition
  bool motorPartCompleted;
  bool warningCalled, buzzerCalled, redLEDCalled, yellowLEDCalled, greenLEDCalled;
  String warningMessage;
} struct_message;

struct_message myData;

float outputBeakerThreshold = 10.0;

// Gyroscope-----------------------------------------------
MPU6050 mpu;

// ULTRASONIC Variables-------------------------------------
const int trigPinUS = 12;
const int echoPinUS = 13;

// Variables to store the duration of the pulse and the distance
long duration;
float distance;

// Buzzer and LED Pins---------------------------------------
#define buzzerPin 33
#define RED_LED_PIN 26
#define GREEN_LED_PIN 16
#define YELLOW_LED_PIN 25

// Define sound patterns
const int EMERGENCY_SOUND = 0;
const int AFTER_WORK_SOUND = 1;
const int CLEANING_TIME_SOUND = 2;

// Water Variables------------------------------------------
bool waterFilled = false;

// Motor Part Variables---------------------------------------
// Define motor control pins for Motor A and B (connected to the first motor driver)
int ena = 19;  // Motor A enable pin
int in1 = 18;  // Motor A input 1
int in2 = 5;   // Motor A input 2
int in3 = 4;   // Motor B input 1
int in4 = 2;   // Motor B input 2
int enb = 15;  // Motor B enable pin

Servo motorC;  // Create a Servo object for motor C
Servo motorD;  // Create a Servo object for motor D

const int motorCPin = 27;  // Define the pin number for motor C
const int motorDPin = 14;  // Define the pin number for motor D

unsigned long previousMillis = 0;     // Store the last time a motor action was triggered
unsigned long stateChangeMillis = 0;  // Store the last time the state was changed

// Timing constants for Motor A
const unsigned long motorARotationDuration = 350;  // Duration of each Motor A rotation in milliseconds
const unsigned long motorAStopDuration = 1000;     // Stop duration after each Motor A rotation in milliseconds

// Timing constants for Motor B
const unsigned long motorBRotationDuration = 250;  // Duration of each Motor B rotation in milliseconds
const unsigned long motorBStopDuration = 1000;     // Stop duration after each Motor B rotation in milliseconds

// Timing constants for Servos
const unsigned long servoMoveDuration = 1000;  // Duration for servo movements

// Timing constant for state transition gap
const unsigned long stateTransitionGap = 1000;  // 1500 milliseconds gap between state transitions

// Combined motor states
enum MotorState {
  MOTOR_A_COUNTERCLOCKWISE_01,
  MOTOR_A_COUNTERCLOCKWISE_01_STOP,
  MOTOR_B_CLOCKWISE_01,
  MOTOR_B_CLOCKWISE_01_STOP,
  MOTOR_C_RESET_01,
  MOTOR_D_ROTATET_01,
  MOTOR_C_ROTATET_01,
  MOTOR_B_COUNTERCLOCKWISE_01,
  MOTOR_B_COUNTERCLOCKWISE_01_STOP,
  MOTOR_A_CLOCKWISE_01,
  MOTOR_A_CLOCKWISE_01_STOP,
  MOTOR_B_CLOCKWISE_02,
  MOTOR_B_CLOCKWISE_02_STOP,
  MOTOR_C_RESET_02,
  MOTOR_D_RESET_02,
  MOTOR_C_ROTATET_02,
  MOTOR_B_COUNTERCLOCKWISE_02,
  MOTOR_B_COUNTERCLOCKWISE_02_STOP,
  MOTOR_A_CLOCKWISE_02,
  MOTOR_A_CLOCKWISE_02_STOP,
  MOTOR_B_CLOCKWISE_03,
  MOTOR_B_CLOCKWISE_03_STOP,
  MOTOR_C_RESET_03,
  MOTOR_D_ROTATET_03,
  MOTOR_C_ROTATET_03,
  MOTOR_B_COUNTERCLOCKWISE_03,
  MOTOR_B_COUNTERCLOCKWISE_03_STOP,
  MOTOR_A_COUNTERCLOCKWISE_03,
  MOTOR_A_COUNTERCLOCKWISE_03_STOP,
  MOTOR_B_CLOCKWISE_04,
  MOTOR_B_CLOCKWISE_04_STOP,
  MOTOR_C_RESET_04,
  MOTOR_D_RESET_04,
  MOTOR_C_ROTATET_04,
  MOTOR_B_COUNTERCLOCKWISE_04,
  MOTOR_B_COUNTERCLOCKWISE_04_STOP,
  MOTOR_A_COUNTERCLOCKWISE_04,
  MOTOR_A_COUNTERCLOCKWISE_04_STOP,
  MOTOR_B_CLOCKWISE_05,
  MOTOR_B_CLOCKWISE_05_STOP,
  MOTOR_C_RESET_05,
  MOTOR_D_ROTATET_05,
  MOTOR_C_ROTATET_05,
  MOTOR_B_COUNTERCLOCKWISE_05,
  MOTOR_B_COUNTERCLOCKWISE_05_STOP,
  MOTOR_A_CLOCKWISE_05,
  MOTOR_A_CLOCKWISE_05_STOP,
  MOTOR_B_CLOCKWISE_06,
  MOTOR_B_CLOCKWISE_06_STOP,
  MOTOR_C_RESET_06,
  MOTOR_D_RESET_06,
  MOTOR_C_ROTATET_06,
  MOTOR_B_COUNTERCLOCKWISE_06,
  MOTOR_B_COUNTERCLOCKWISE_06_STOP,
  MOTOR_A_COUNTERCLOCKWISE_06,
  MOTOR_A_COUNTERCLOCKWISE_06_STOP,
  MOTOR_B_CLOCKWISE_07,
};

MotorState currentState = MOTOR_A_COUNTERCLOCKWISE_01;  // Initialize the state machine with the first state

// Function Declarations--------------------------------------
void initializeComponents();
void runGyroscope();
void runUltrasonicOutputCheck();
void runMotorPart();
void shortRepeatedBeeps();
void playReversingSound();
void longBeepWithShortPauses();
void tone(int pin, int frequency);
void noTone(int pin);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len);
void sendData();
void warning(const char *message);

void setup() {
  Serial.begin(115200);
  initializeComponents();
}

void loop() {
  bool gyroscopeFinishedPrinted = false;
  bool ultrasonicCheckCalled = false;
  String warningMessage = myData.warningMessage;

  if (!myData.gyroscopeEnterPressed) {
    runGyroscope();
  }

  if (myData.outputCheckCalled && !myData.motorPartCompleted) {
    // run ultrasonic sensor to check output beaker
    Serial.println("Final output was displayed and output beaker check called");
    runUltrasonicOutputCheck();
  }

  if (waterFilled) {
    // run motor part
    Serial.println("TOF detected threshold and motor part called");
    runMotorPart();
  }

  if (myData.warningCalled) {
    Serial.println(warningMessage);
    warning(warningMessage.c_str());
  }
}

void initializeComponents() {
  // ESP32 Initialization
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, espBoardA_address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Gyroscope Initialization
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }

  // Buzzer Initialization
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // Ultrasonic Sensor Initialization
  pinMode(trigPinUS, OUTPUT);
  pinMode(echoPinUS, INPUT);

  // Motor pins initialization
  // Set motor control pins as outputs
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enb, OUTPUT);

  // Attach servo motors to the defined pins
  motorC.attach(motorCPin);
  motorD.attach(motorDPin);

  // Initial motor states
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(ena, 0);
  analogWrite(enb, 0);

  myData.motorPartCompleted = false;
}

// Function to read and process gyroscope data
void runGyroscope() {
  Serial.println("Gyroscope Running");
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  int angleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  int angleY = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  int angleZ = atan2(sqrt(accelX * accelX + accelY * accelY), accelZ) * 180.0 / PI;

  Serial.print("X: ");
  Serial.print(angleX);
  Serial.print(" degrees\t");

  Serial.print("Y: ");
  Serial.print(angleY);
  Serial.print(" degrees\t");

  Serial.print("Z: ");
  Serial.print(angleZ);
  Serial.println(" degrees");

  myData.angleX = angleX;
  myData.angleY = angleY;
  myData.angleZ = angleZ;
  myData.gyroscopeDataSent = true;

  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime > 1000) {
    sendData();
    lastSendTime = currentTime;
  }

  // Reset the gyroscope data flag after sending
  myData.gyroscopeDataSent = false;

  delay(1000);
}

void runUltrasonicOutputCheck() {
  digitalWrite(trigPinUS, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinUS, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinUS, LOW);

  duration = pulseIn(echoPinUS, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= outputBeakerThreshold) {
    myData.ultrasonicObjectDetected = true;
    Serial.println("Output Beaker Detected");
  } else {
    warning("No Output Beaker");
  }

  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime > 1000) {
    sendData();
    lastSendTime = currentTime;
  }

  delay(1000);
}

void runMotorPart() {
  // starts the mechanical part
  Serial.println("Motor Working");


  digitalWrite(26, HIGH);
  delay(1000);
  digitalWrite(26, LOW);
  delay(1000);



  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena, 255);
  delay(2000);

  //STOP
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(2000);

  //COUNTERCLOCKWISE MAX SPEED
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena, 255);
  delay(2000);

  //STOP
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(2000);

  //CLOCKWISE HALF SPEED
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena, 255);
  delay(2000);

  //STOP
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(2000);

  //COUNTERCLOCKWISE MAX SPEED
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena, 255);
  delay(2000);

  //STOP
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  //-----------------------------------------------------------------------
  // Set the motor part completed flag
  myData.motorPartCompleted = true;

  // Send the data to the other board
  sendData();

  // Call the success function locally
  success();
}

void Buzzer(int pattern) {
  switch (pattern) {
    case EMERGENCY_SOUND:
      shortRepeatedBeeps();
      break;
    case AFTER_WORK_SOUND:
      playReversingSound();
      break;
    case CLEANING_TIME_SOUND:
      longBeepWithShortPauses();
      break;
    default:
      Serial.println("Unknown sound pattern!");
  }
}

// Function to play short repeated beeps (emergency sound)
void shortRepeatedBeeps() {
  int toneFrequency = 1000;  // Frequency of the tone in Hz
  int toneDuration = 100;    // Duration of each tone in milliseconds
  int pauseDuration = 100;   // Pause between tones in milliseconds

  for (int i = 0; i < 5; i++) {      // Play 5 beeps
    tone(buzzerPin, toneFrequency);  // Play tone
    delay(toneDuration);             // Wait for tone duration
    noTone(buzzerPin);               // Stop the tone
    delay(pauseDuration);            // Wait between beeps
  }
}

// Function to play the reversing sound (after work sound)
void playReversingSound() {
  int toneFrequency = 1000;  // Frequency of the tone in Hz
  int toneDuration = 750;    // Duration of each tone in milliseconds
  int pauseDuration = 500;   // Pause between tones in milliseconds

  for (int i = 0; i < 5; i++) {      // Play 5 beeps
    tone(buzzerPin, toneFrequency);  // Play tone
    delay(toneDuration);             // Wait for tone duration
    noTone(buzzerPin);               // Stop the tone
    delay(pauseDuration);            // Wait between beeps
  }
}

// Function to play long beep with short pauses (cleaning time sound)
void longBeepWithShortPauses() {
  int toneFrequency = 800;  // Frequency of the tone in Hz
  int toneDuration = 500;   // Duration of each tone in milliseconds
  int pauseDuration = 200;  // Pause between tones in milliseconds

  for (int i = 0; i < 3; i++) {      // Play 3 long beeps
    tone(buzzerPin, toneFrequency);  // Play tone
    delay(toneDuration);             // Wait for tone duration
    noTone(buzzerPin);               // Stop the tone
    delay(pauseDuration);            // Wait between beeps
  }
}

// Tone and noTone functions for ESP32 (if not using the standard library)
void tone(int pin, int frequency) {
  // Configure LEDC timer and channel
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = (uint32_t)frequency,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
    .gpio_num = pin,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 127,  // 50% duty cycle
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channel);
  ledc_timer_pause(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);
  ledc_timer_resume(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0);
}

void noTone(int pin) {
  ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);  // Stop the PWM signal
}

// Data Communication---------------------------------------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\nLast Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);

  if (myData.warningCalled) {
    warning(myData.warningMessage.c_str());  // Call warning function if warning flag is set
  }
  if (myData.waterFilled) {
    // run motor part
    Serial.println("TOF detected threshold and motor part called");
    waterFilled = true;
  }
}

void sendData() {
  esp_err_t result = esp_now_send(espBoardA_address, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  // Reset the gyroscope data flag after sending
  myData.gyroscopeDataSent = false;
  // Reset the motor part completed flag after sending
  myData.motorPartCompleted = false;
  // Reset the warning flag after sending
  myData.warningCalled = false;
}

// Warning Function
void warning(const char *message) {
  // Set the warning flag and message
  myData.warningCalled = true;
  myData.warningMessage = message;

  // Send the data to the other board
  sendData();

  // Activate buzzer and LED
  Buzzer(EMERGENCY_SOUND);
  digitalWrite(RED_LED_PIN, HIGH);

  // Display warning message on Serial Monitor
  Serial.print("Warning: ");
  Serial.println(message);

  // Send warning message to the other ESP32 board
  struct_message warningData = myData;
  myData.warningMessage = message;
  esp_now_send(espBoardA_address, (uint8_t *)&warningData, sizeof(warningData));

  // Keep the warning state for 5 seconds
  delay(5000);

  // Deactivate LED
  digitalWrite(buzzerPin, LOW);

  // turn off the warning flag
  myData.warningCalled = false;

  // Return to previous state or reset as needed
  // resetCircuit(); // Uncomment if you have a reset function
}

void success() {
  // Activate buzzer and LED
  Buzzer(AFTER_WORK_SOUND);
  digitalWrite(GREEN_LED_PIN, HIGH);

  // Keep the success state for 5 seconds
  delay(5000);

  // Deactivate buzzer and LED
  digitalWrite(GREEN_LED_PIN, LOW);
}