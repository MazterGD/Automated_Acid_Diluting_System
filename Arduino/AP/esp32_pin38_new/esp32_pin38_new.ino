#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

// ESP32------------------------------------------------
// MAC address of the first ESP32 (esp32_1)
uint8_t esp32_1_address[] = { 0xA0, 0xA3, 0xB3, 0x2A, 0xDE, 0x3C };  // Replace with actual MAC address of esp32_1

// Structure to hold data to be sent
typedef struct struct_message {
  int angleX, angleY, angleZ;  // Gyroscope data
  // bool gyroscopeEnterPressed;  // Enter is pressed when Gyroscope is displayed
  // bool buzzerCalled;
  // bool tofThresholdDetected;
  // bool ultrasonicObjectDetected;
} struct_message;

struct_message myData;

// Gyroscope------------------------------------------------
MPU6050 mpu;

// // ULTRASONIC Variables-------------------------------------
// const int trigPinUS2 = 12;
// const int echoPinUS2 = 13;

// // Variables to store the duration of the pulse and the distance
// long duration2;
// int waterLevel;

// // Buzzer Variables-----------------------------------------
// const int buzzerPin = 33; // Pin connected to the buzzer

// // Motor Part-----------------------------------------------
// // Define motor control pins for Motor A and B (connected to the first motor driver)
// int ena = 15;   // Motor A enable pin
// int in1 = 2;    // Motor A input 1
// int in2 = 4;    // Motor A input 2
// int in3 = 5;    // Motor B input 1
// int in4 = 18;   // Motor B input 2
// int enb = 19;   // Motor B enable pin

// Servo motorC; // Create a Servo object for motor C
// Servo motorD; // Create a Servo object for motor D

// const int motorCPin = 27; // Define the pin number for motor C
// const int motorDPin = 14; // Define the pin number for motor D

// unsigned long previousMillis = 0; // Store the last time a motor action was triggered
// unsigned long stateChangeMillis = 0; // Store the last time the state was changed

// // Timing constants for Motor A
// const unsigned long motorARotationDuration = 300;  // Duration of each Motor A rotation in milliseconds
// const unsigned long motorAStopDuration = 1000;     // Stop duration after each Motor A rotation in milliseconds

// // Timing constants for Motor B
// const unsigned long motorBRotationDuration = 200;  // Duration of each Motor B rotation in milliseconds
// const unsigned long motorBStopDuration = 1000;     // Stop duration after each Motor B rotation in milliseconds

// // Timing constants for Servos
// const unsigned long servoMoveDuration = 1000;      // Duration for servo movements

// // Timing constant for state transition gap
// const unsigned long stateTransitionGap = 1000; // 1500 milliseconds gap between state transitions

// // Combined motor states
// enum MotorState {
//   MOTOR_B_CLOCKWISE_01,
//   MOTOR_B_CLOCKWISE_01_STOP,
//   MOTOR_C_RESET_01,
//   MOTOR_D_ROTATET_01,
//   MOTOR_C_ROTATET_01,
//   MOTOR_B_COUNTERCLOCKWISE_01,
//   MOTOR_B_COUNTERCLOCKWISE_01_STOP,
//   MOTOR_A_CLOCKWISE_01,
//   MOTOR_A_CLOCKWISE_01_STOP,
//   MOTOR_B_CLOCKWISE_02,
//   MOTOR_B_CLOCKWISE_02_STOP,
//   MOTOR_C_RESET_02,
//   MOTOR_D_RESET_02,
//   MOTOR_C_ROTATET_02,
//   MOTOR_B_COUNTERCLOCKWISE_02,
//   MOTOR_B_COUNTERCLOCKWISE_02_STOP,
//   MOTOR_A_CLOCKWISE_02,
//   MOTOR_A_CLOCKWISE_02_STOP,
//   MOTOR_B_CLOCKWISE_03,
//   MOTOR_B_CLOCKWISE_03_STOP,
//   MOTOR_C_RESET_03,
//   MOTOR_D_ROTATET_03,
//   MOTOR_C_ROTATET_03,
//   MOTOR_B_COUNTERCLOCKWISE_03,
//   MOTOR_B_COUNTERCLOCKWISE_03_STOP,
//   MOTOR_A_COUNTERCLOCKWISE_03,
//   MOTOR_A_COUNTERCLOCKWISE_03_STOP,
//   MOTOR_B_CLOCKWISE_04,
//   MOTOR_B_CLOCKWISE_04_STOP,
//   MOTOR_C_RESET_04,
//   MOTOR_D_RESET_04,
//   MOTOR_C_ROTATET_04,
//   MOTOR_B_COUNTERCLOCKWISE_04,
//   MOTOR_B_COUNTERCLOCKWISE_04_STOP,
//   MOTOR_A_COUNTERCLOCKWISE_04,
//   MOTOR_A_COUNTERCLOCKWISE_04_STOP,
//   MOTOR_B_CLOCKWISE_05,
//   MOTOR_B_CLOCKWISE_05_STOP,
//   MOTOR_C_RESET_05,
//   MOTOR_D_ROTATET_05,
//   MOTOR_C_ROTATET_05,
//   MOTOR_B_COUNTERCLOCKWISE_05,
//   MOTOR_B_COUNTERCLOCKWISE_05_STOP,
//   MOTOR_A_CLOCKWISE_05,
//   MOTOR_A_CLOCKWISE_05_STOP,
//   MOTOR_B_CLOCKWISE_06,
//   MOTOR_B_CLOCKWISE_06_STOP,
//   MOTOR_C_RESET_06,
//   MOTOR_D_RESET_06,
//   MOTOR_C_ROTATET_06,
//   MOTOR_B_COUNTERCLOCKWISE_06,
//   MOTOR_B_COUNTERCLOCKWISE_06_STOP,
//   MOTOR_A_COUNTERCLOCKWISE_06,
//   MOTOR_A_COUNTERCLOCKWISE_06_STOP,
//   MOTOR_B_CLOCKWISE_07,
// };

// MotorState currentState = MOTOR_B_CLOCKWISE_01; // Initialize the state machine with the first state

// Functions-----------------------------------------------
void runGyroscope() {
  // Get accelerometer and gyroscope data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert accelerometer values to G's (divide by 16384, as the range is ±2g)
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  // Calculate the tilt angles
  int angleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  int angleY = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  int angleZ = atan2(sqrt(accelX * accelX + accelY * accelY), accelZ) * 180.0 / PI;

  Serial.print("X: ");
  Serial.print(angleX);  // Make the default value 0
  Serial.print(" degrees\t");

  Serial.print("Y: ");
  Serial.print(angleY);  // Make the default value 0
  Serial.print(" degrees\t");

  Serial.print("Z: ");
  Serial.print(angleZ);  // Make the default value 0
  Serial.println(" degrees");

  // Update gyroscope data in myData
  myData.angleX = angleX;
  myData.angleY = angleY;
  myData.angleZ = angleZ;

  delay(1000);
}

// // Liquid Level & Beaker detecting Ultrasonic
// void runUltrasonic2() {
//   // Clear the trigPin by setting it LOW
//   digitalWrite(trigPinUS2, LOW);
//   delayMicroseconds(2);

//   // Trigger the sensor by setting the trigPin HIGH for 10 microseconds
//   digitalWrite(trigPinUS2, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPinUS2, LOW);

//   // Read the echoPin, which returns the duration of the pulse in microseconds
//   duration2 = pulseIn(echoPinUS2, HIGH);

//   // Calculate the distance in centimeters
//   waterLevel = duration2 * 0.034 / 2;

//   // Print the distance to the Serial Monitor
//   Serial.print("Water Level: ");
//   Serial.print(waterLevel);
//   Serial.println(" cm");

//   // Update distance data in myData
//   myData.waterLevel = waterLevel;

//   delay(1000);
// }

// void runMotor() {
//   unsigned long currentMillis = millis();

//   switch (currentState) {
//     case MOTOR_B_CLOCKWISE_01:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, HIGH);
//         digitalWrite(in4, LOW);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_01_STOP;
//       }
//       break;
//     case MOTOR_B_CLOCKWISE_01_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_C_RESET_01;
//       }
//       break;
//     case MOTOR_C_RESET_01:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorC.write(0);
//         previousMillis = currentMillis;
//         currentState = MOTOR_D_ROTATET_01;
//         delay(1000);
//       }
//       break;
//     case MOTOR_D_ROTATET_01:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorD.write(180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_C_ROTATET_01;
//         delay(1000);
//       }
//       break;
//     case MOTOR_C_ROTATET_01:
//       if (currentMillis - previousMillis >= servoMoveDuration) {
//         motorC.write(180);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_01;
//         delay(1000);
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_01:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, LOW);
//         digitalWrite(in4, HIGH);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_01_STOP;
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_01_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_A_CLOCKWISE_01;
//       }
//       break;
//     case MOTOR_A_CLOCKWISE_01:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in1, HIGH);
//         digitalWrite(in2, LOW);
//         analogWrite(ena, 230);
//         previousMillis = currentMillis;
//         currentState = MOTOR_A_CLOCKWISE_01_STOP;
//       }
//       break;
//     case MOTOR_A_CLOCKWISE_01_STOP:
//       if (currentMillis - previousMillis >= motorARotationDuration) {
//         analogWrite(ena, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_02;
//       }
//       break;
// //-----------------------------------------------------------------------
//     // Add similar cases for other states
//     case MOTOR_B_CLOCKWISE_02:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, HIGH);
//         digitalWrite(in4, LOW);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_02_STOP;
//       }
//       break;
//     case MOTOR_B_CLOCKWISE_02_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_C_RESET_02;
//       }
//       break;
//     case MOTOR_C_RESET_02:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorC.write(0);
//         previousMillis = currentMillis;
//         currentState = MOTOR_D_RESET_02;
//         delay(1000);
//       }
//       break;
//     case MOTOR_D_RESET_02:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorD.write(0);
//         previousMillis = currentMillis;
//         currentState = MOTOR_C_ROTATET_02;
//         delay(1000);
//       }
//       break;
//     case MOTOR_C_ROTATET_02:
//       if (currentMillis - previousMillis >= servoMoveDuration) {
//         motorC.write(180);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_02;
//         delay(1000);
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_02:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, LOW);
//         digitalWrite(in4, HIGH);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_02_STOP;
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_02_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_A_CLOCKWISE_02;
//       }
//       break;
//     case MOTOR_A_CLOCKWISE_02:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in1, HIGH);
//         digitalWrite(in2, LOW);
//         analogWrite(ena, 230);
//         previousMillis = currentMillis;
//         currentState = MOTOR_A_CLOCKWISE_02_STOP;
//       }
//       break;
//     case MOTOR_A_CLOCKWISE_02_STOP:
//       if (currentMillis - previousMillis >= 2000) {
//         analogWrite(ena, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_03;
//       }
//       break;
// //-----------------------------------------------------------------------
      
//     case MOTOR_B_CLOCKWISE_03:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, HIGH);
//         digitalWrite(in4, LOW);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_03_STOP;
//       }
//       break;
//     case MOTOR_B_CLOCKWISE_03_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_C_RESET_03;
//       }
//       break;
//     case MOTOR_C_RESET_03:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorC.write(0);
//         previousMillis = currentMillis;
//         currentState = MOTOR_D_ROTATET_03;
//         delay(1000);
//       }
//       break;
//     case MOTOR_D_ROTATET_03:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorD.write(180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_C_ROTATET_03;
//         delay(1000);
//       }
//       break;
//     case MOTOR_C_ROTATET_03:
//       if (currentMillis - previousMillis >= servoMoveDuration) {
//         motorC.write(180);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_03;
//         delay(1000);
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_03:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, LOW);
//         digitalWrite(in4, HIGH);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_03_STOP;
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_03_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_A_COUNTERCLOCKWISE_03; // Changed state name
//       }
//       break;
//     case MOTOR_A_COUNTERCLOCKWISE_03: // New state name
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in1, LOW); // Set motor A to rotate counterclockwise
//         digitalWrite(in2, HIGH); // Set motor A to rotate counterclockwise
//         analogWrite(ena, 230);
//         previousMillis = currentMillis;
//         currentState = MOTOR_A_COUNTERCLOCKWISE_03_STOP; // Changed state name
//       }
//       break;
//     case MOTOR_A_COUNTERCLOCKWISE_03_STOP: // New state name
//       if (currentMillis - previousMillis >= motorARotationDuration) {
//         analogWrite(ena, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_04;
//       }
//       break;

//       //--------------------------------------------------------------------
//         case MOTOR_B_CLOCKWISE_04:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, HIGH);
//         digitalWrite(in4, LOW);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_04_STOP;
//       }
//       break;
//     case MOTOR_B_CLOCKWISE_04_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_C_RESET_04;
//       }
//       break;
//     case MOTOR_C_RESET_04:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorC.write(0);
//         previousMillis = currentMillis;
//         currentState = MOTOR_D_RESET_04;
//         delay(1000);
//       }
//       break;
//     case MOTOR_D_RESET_04:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorD.write(0);
//         previousMillis = currentMillis;
//         currentState = MOTOR_C_ROTATET_04;
//         delay(1000);
//       }
//       break;
//     case MOTOR_C_ROTATET_04:
//       if (currentMillis - previousMillis >= servoMoveDuration) {
//         motorC.write(180);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_04;
//         delay(1000);
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_04:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, LOW);
//         digitalWrite(in4, HIGH);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_04_STOP;
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_04_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_A_COUNTERCLOCKWISE_04; // Changed state name
//       }
//       break;
//     case MOTOR_A_COUNTERCLOCKWISE_04: // New state name
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in1, LOW); // Set motor A to rotate counterclockwise
//         digitalWrite(in2, HIGH); // Set motor A to rotate counterclockwise
//         analogWrite(ena, 230);
//         previousMillis = currentMillis;
//         currentState = MOTOR_A_COUNTERCLOCKWISE_04_STOP; // Changed state name
//       }
//       break;
//     case MOTOR_A_COUNTERCLOCKWISE_04_STOP: // New state name
//       if (currentMillis - previousMillis >= motorARotationDuration) {
//         analogWrite(ena, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_05;
//       }
//       break;

//     //--------------------------------------------------------------------

//     case MOTOR_B_CLOCKWISE_05:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, HIGH);
//         digitalWrite(in4, LOW);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_05_STOP;
//       }
//       break;
//     case MOTOR_B_CLOCKWISE_05_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_C_RESET_05;
//       }
//       break;
//     case MOTOR_C_RESET_05:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorC.write(0);
//         previousMillis = currentMillis;
//         currentState = MOTOR_D_ROTATET_05;
//         delay(1000);
//       }
//       break;
//     case MOTOR_D_ROTATET_05:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorD.write(180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_C_ROTATET_05;
//         delay(1000);
//       }
//       break;
//     case MOTOR_C_ROTATET_05:
//       if (currentMillis - previousMillis >= servoMoveDuration) {
//         motorC.write(180);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_05;
//         delay(1000);
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_05:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, LOW);
//         digitalWrite(in4, HIGH);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_05_STOP;
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_05_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_A_CLOCKWISE_05;
//       }
//       break;
//     case MOTOR_A_CLOCKWISE_05:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in1, HIGH);
//         digitalWrite(in2, LOW);
//         analogWrite(ena, 230);
//         previousMillis = currentMillis;
//         currentState = MOTOR_A_CLOCKWISE_05_STOP;
//       }
//       break;
//     case MOTOR_A_CLOCKWISE_05_STOP:
//       if (currentMillis - previousMillis >= motorARotationDuration) {
//         analogWrite(ena, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_06;
//       }
//       break;

//       //------------------------------------------------------------------

//       case MOTOR_B_CLOCKWISE_06:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, HIGH);
//         digitalWrite(in4, LOW);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_06_STOP;
//       }
//       break;
//     case MOTOR_B_CLOCKWISE_06_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_C_RESET_06;
//       }
//       break;
//     case MOTOR_C_RESET_06:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorC.write(0);
//         previousMillis = currentMillis;
//         currentState = MOTOR_D_RESET_06;
//         delay(1000);
//       }
//       break;
//     case MOTOR_D_RESET_06:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         motorD.write(0);
//         previousMillis = currentMillis;
//         currentState = MOTOR_C_ROTATET_06;
//         delay(1000);
//       }
//       break;
//     case MOTOR_C_ROTATET_06:
//       if (currentMillis - previousMillis >= servoMoveDuration) {
//         motorC.write(180);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_06;
//         delay(1000);
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_06:
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in3, LOW);
//         digitalWrite(in4, HIGH);
//         analogWrite(enb, 180);
//         previousMillis = currentMillis;
//         currentState = MOTOR_B_COUNTERCLOCKWISE_06_STOP;
//       }
//       break;
//     case MOTOR_B_COUNTERCLOCKWISE_06_STOP:
//       if (currentMillis - previousMillis >= motorBRotationDuration) {
//         analogWrite(enb, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_A_COUNTERCLOCKWISE_06; // Changed state name
//       }
//       break;
//     case MOTOR_A_COUNTERCLOCKWISE_06: // New state name
//       if (currentMillis - stateChangeMillis >= stateTransitionGap) {
//         digitalWrite(in1, LOW); // Set motor A to rotate counterclockwise
//         digitalWrite(in2, HIGH); // Set motor A to rotate counterclockwise
//         analogWrite(ena, 230);
//         previousMillis = currentMillis;
//         currentState = MOTOR_A_COUNTERCLOCKWISE_06_STOP; // Changed state name
//       }
//       break;
//     case MOTOR_A_COUNTERCLOCKWISE_06_STOP: // New state name
//       if (currentMillis - previousMillis >= 2000) {
//         analogWrite(ena, 0);
//         previousMillis = currentMillis;
//         stateChangeMillis = currentMillis;
//         currentState = MOTOR_B_CLOCKWISE_07;
//       }
//       break;
//   }
// }

// // Function to play short repeated beeps
// void shortRepeatedBeeps() {
//   int toneFrequency = 1000; // Frequency of the tone in Hz
//   int toneDuration = 100;   // Duration of each tone in milliseconds
//   int pauseDuration = 100;  // Pause between tones in milliseconds

//   for (int i = 0; i < 5; i++) { // Play 5 beeps
//     tone(buzzerPin, toneFrequency); // Play tone
//     delay(toneDuration);           // Wait for tone duration
//     noTone(buzzerPin);             // Stop the tone
//     delay(pauseDuration);          // Wait between beeps
//   }
// }

// Data Communication---------------------------------------
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
  // Send data to first ESP32
  esp_err_t result = esp_now_send(esp32_1_address, (uint8_t *)&myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

void setup() {
  // Serial Monitor-----------------------------------
  Serial.begin(115200);

  // ESP32--------------------------------------------
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // Add peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, esp32_1_address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // // Ultrasonic---------------------------------------
  // // Set the ultrasonic sensor pins as output and input
  // pinMode(trigPinUS2, OUTPUT);
  // pinMode(echoPinUS2, INPUT);

  // Gyroscope----------------------------------------
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // // Buzzer---------------------------------------------
  // pinMode(buzzerPin, OUTPUT); // Set buzzer pin as output

  // // Motor Part---------------------------------------
  // // Set motor control pins as outputs
  // pinMode(ena, OUTPUT);
  // pinMode(in1, OUTPUT);
  // pinMode(in2, OUTPUT);
  // pinMode(in3, OUTPUT);
  // pinMode(in4, OUTPUT);
  // pinMode(enb, OUTPUT);

  // // Attach servo motors to the defined pins
  // motorC.attach(motorCPin);
  // motorD.attach(motorDPin);
}

void loop() {
  runGyroscope();

  // if (myData.ultrasonicObjectDetected) {
  //   runMotor();
  //   runUltrasonic2();
  // }

  // if (myData.buzzerCalled) {
  //   shortRepeatedBeeps();
  //   Serial.println("FUcked");
  // }
  
  // Optionally control sendData() frequency
  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime > 1000) {  // Send data every 1 second
    sendData();
    lastSendTime = currentTime;
  }
}