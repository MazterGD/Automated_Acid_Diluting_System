#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Arduino.h>
#include <driver/ledc.h> // Explicitly include the LED control library

// ESP32------------------------------------------------------
uint8_t espBoardB_address[] = { 0xA0, 0xA3, 0xB3, 0x2A, 0xDE, 0x3C };  // Replace with actual MAC address of the receiver ESP32

// Structure to hold data to be sent
typedef struct struct_message {
  int angleX, angleY, angleZ;  // Gyroscope data
  bool finalOutputIsDisplayed;
  bool ultrasonicObjectDetected;
  bool tofThresholdDetected;   // water part completion condition
} struct_message;

struct_message myData;

float outputBeakerThreshold = 10.0;

// Gyroscope-----------------------------------------------
MPU6050 mpu;

// ULTRASONIC Variables-------------------------------------
const int trigPinUS = 12;
const int echoPinUS = 13;
long duration;
float distance;

// Buzzer Pin
const int BUZZER_PIN = 15; // Define the pin connected to the buzzer

// Buzzer Constants
#define EMERGENCY_SOUND 1
#define AFTER_WORK_SOUND 2
#define CLEANING_TIME_SOUND 3

// Enumeration for states
enum State {
  INITIAL_STATE,
  GYROSCOPE_STATE,
  ULTRASONIC_OUTPUT_CHECK_STATE,
  MOTOR_PART_STATE,
  // Add more states as needed
};
State currentState = INITIAL_STATE;

// Function Declarations
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
void warning(const char* message);

void setup() {
  Serial.begin(115200);
  initializeComponents();
}

void loop() {
  switch (currentState) {
    case INITIAL_STATE:
      // Add any initial state actions here
      Serial.println("Initial State");
      currentState = GYROSCOPE_STATE; // Transition to the next state
      break;

    case GYROSCOPE_STATE:
    Serial.println("Gyroscope State");
      runGyroscope();
      sendData();
      currentState = ULTRASONIC_OUTPUT_CHECK_STATE; // Transition to the next state
      break;

    case ULTRASONIC_OUTPUT_CHECK_STATE:
      if (myData.finalOutputIsDisplayed) {
        runUltrasonicOutputCheck();
      }
      if (myData.ultrasonicObjectDetected) {
        currentState = MOTOR_PART_STATE; // Transition to the next state
      }
      break;

    case MOTOR_PART_STATE:
      if (myData.tofThresholdDetected) {
        runMotorPart();
      }
      // Transition to other states or repeat as needed
      break;

    // Add more cases for additional states as needed
    default:
      break;
  }

  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime > 10000) {
    sendData();
    lastSendTime = currentTime;
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
  memcpy(peerInfo.peer_addr, espBoardB_address, 6);
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
    while (1);
  }

  // Buzzer Initialization
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Ultrasonic Sensor Initialization
  pinMode(trigPinUS, OUTPUT);
  pinMode(echoPinUS, INPUT);
}

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
    warning("Output Beaker not detected");
  }

  delay(1000);
}

void runMotorPart() {
  Serial.println("Motor Working");
}

// Warning Function
void warning(const char* message) {
  // Activate buzzer and LED
  digitalWrite(BUZZER_PIN, HIGH);
  // Add your code to turn on the LED if needed

  // Display warning message on Serial Monitor
  Serial.print("Warning: ");
  Serial.println(message);

  // Send warning message to the other ESP32 board
  struct_message warningData = myData;
  warningData.finalOutputIsDisplayed = true; // Indicate a warning state
  esp_now_send(espBoardB_address, (uint8_t *)&warningData, sizeof(warningData));

  // Keep the warning state for 5 seconds
  delay(5000);

  // Deactivate buzzer and LED
  digitalWrite(BUZZER_PIN, LOW);
  // Add your code to turn off the LED if needed

  // Return to previous state or reset as needed
  // resetCircuit(); // Uncomment if you have a reset function
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\nLast Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
}

void sendData() {
  esp_err_t result = esp_now_send(espBoardB_address, (uint8_t *)&myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}