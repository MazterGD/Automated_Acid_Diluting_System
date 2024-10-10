#include <ESP32Servo.h>

// Define motor control pins for Motor A and B (connected to the first motor driver)
int ena = 15;   // Motor A enable pin
int in1 = 2;    // Motor A input 1
int in2 = 4;    // Motor A input 2
int in3 = 5;    // Motor B input 1
int in4 = 18;   // Motor B input 2
int enb = 19;   // Motor B enable pin

Servo motorC; // Create a Servo object for motor C
Servo motorD; // Create a Servo object for motor D

const int motorCPin = 27; // Define the pin number for motor C
const int motorDPin = 14; // Define the pin number for motor D

unsigned long previousMillis = 0; // Store the last time a motor action was triggered
unsigned long stateChangeMillis = 0; // Store the last time the state was changed

// Timing constants for Motor A
const unsigned long motorARotationDuration = 350;  // Duration of each Motor A rotation in milliseconds
const unsigned long motorAStopDuration = 1000;     // Stop duration after each Motor A rotation in milliseconds

// Timing constants for Motor B
const unsigned long motorBRotationDuration = 200;  // Duration of each Motor B rotation in milliseconds
const unsigned long motorBStopDuration = 1000;     // Stop duration after each Motor B rotation in milliseconds

// Timing constants for Servos
const unsigned long servoMoveDuration = 1000;      // Duration for servo movements

// Timing constant for state transition gap
const unsigned long stateTransitionGap = 1000; // 1500 milliseconds gap between state transitions

// Combined motor states
enum MotorState {
  p2MOTOR_A_COUNTERCLOCKWISE_01,
  p2MOTOR_A_COUNTERCLOCKWISE_01_STOP,
  p2MOTOR_B_CLOCKWISE_01,
  p2MOTOR_B_CLOCKWISE_01_STOP,
  p2MOTOR_C_RESET_01,
  p2MOTOR_D_ROTATE_01,
  p2MOTOR_C_ROTATE_01,
  p2MOTOR_B_COUNTERCLOCKWISE_01,
  p2MOTOR_B_COUNTERCLOCKWISE_01_STOP,
  p2MOTOR_A_CLOCKWISE_01,
  p2MOTOR_A_CLOCKWISE_01_STOP,
  //----------------------------------
  p2MOTOR_B_CLOCKWISE_02,
  p2MOTOR_B_CLOCKWISE_02_STOP,
  p2MOTOR_C_RESET_02,
  p2MOTOR_D_RESET_02,
  p2MOTOR_C_ROTATE_02,
  p2MOTOR_B_COUNTERCLOCKWISE_02,
  p2MOTOR_B_COUNTERCLOCKWISE_02_STOP,
  p2MOTOR_A_CLOCKWISE_02,
  p2MOTOR_A_CLOCKWISE_02_STOP,
   //----------------------------------
  p2MOTOR_B_CLOCKWISE_03,
  p2MOTOR_B_CLOCKWISE_03_STOP,
  p2MOTOR_C_RESET_03,
  p2MOTOR_D_ROTATE_03,
  p2MOTOR_C_ROTATE_03,
  p2MOTOR_B_COUNTERCLOCKWISE_03,
  p2MOTOR_B_COUNTERCLOCKWISE_03_STOP,
  p2MOTOR_A_CLOCKWISE_03,
  p2MOTOR_A_CLOCKWISE_03_STOP,
   //----------------------------------
  p2MOTOR_B_CLOCKWISE_04,
  p2MOTOR_B_CLOCKWISE_04_STOP,
  p2MOTOR_C_RESET_04,
  p2MOTOR_D_RESET_04,
  p2MOTOR_C_ROTATE_04,
  p2MOTOR_B_COUNTERCLOCKWISE_04,
  p2MOTOR_B_COUNTERCLOCKWISE_04_STOP,
  p2MOTOR_A_CLOCKWISE_04,
  p2MOTOR_A_CLOCKWISE_04_STOP,
   //----------------------------------
  p2MOTOR_B_CLOCKWISE_05,
  p2MOTOR_B_CLOCKWISE_05_STOP,
  p2MOTOR_C_RESET_05,
  p2MOTOR_D_ROTATE_05,
  p2MOTOR_C_ROTATE_05,
  p2MOTOR_B_COUNTERCLOCKWISE_05,
  p2MOTOR_B_COUNTERCLOCKWISE_05_STOP,
  p2MOTOR_A_COUNTERCLOCKWISE_05,
  p2MOTOR_A_COUNTERCLOCKWISE_05_STOP,
  //------------------------------------
  p2MOTOR_B_CLOCKWISE_06,
  p2MOTOR_B_CLOCKWISE_06_STOP,
  p2MOTOR_C_RESET_06,
  p2MOTOR_D_RESET_06,
  p2MOTOR_C_ROTATE_06,
  p2MOTOR_B_COUNTERCLOCKWISE_06,
  p2MOTOR_B_COUNTERCLOCKWISE_06_STOP,
  p2MOTOR_A_CLOCKWISE_06,
  p2MOTOR_A_CLOCKWISE_06_STOP,
   //----------------------------------
  p2MOTOR_B_CLOCKWISE_07,
  p2MOTOR_B_CLOCKWISE_07_STOP,
  p2MOTOR_C_RESET_07,
  p2MOTOR_D_ROTATE_07,
  p2MOTOR_C_ROTATE_07,
  p2MOTOR_B_COUNTERCLOCKWISE_07,
  p2MOTOR_B_COUNTERCLOCKWISE_07_STOP,
  p2MOTOR_A_CLOCKWISE_07,
  p2MOTOR_A_CLOCKWISE_07_STOP,
   //----------------------------------
  p2MOTOR_B_CLOCKWISE_08,
  p2MOTOR_B_CLOCKWISE_08_STOP,
  p2MOTOR_C_RESET_08,
  p2MOTOR_D_RESET_08,
  p2MOTOR_C_ROTATE_08,
  p2MOTOR_B_COUNTERCLOCKWISE_08,
  p2MOTOR_B_COUNTERCLOCKWISE_08_STOP,
  p2MOTOR_A_COUNTERCLOCKWISE_08,
  p2MOTOR_A_COUNTERCLOCKWISE_08_STOP,
   //----------------------------------
  p2MOTOR_B_CLOCKWISE_09,
};

MotorState currentState = p2MOTOR_A_COUNTERCLOCKWISE_01; // Initialize the state machine with the first state

void setup() {
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
}

void loop() {
  unsigned long currentMillis = millis();

  switch (currentState) {
       case p2MOTOR_A_COUNTERCLOCKWISE_01: // New state name
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in1, LOW); // Set motor A to rotate counterclockwise
        digitalWrite(in2, HIGH); // Set motor A to rotate counterclockwise
        analogWrite(ena, 245);
        previousMillis = currentMillis;
        currentState = p2MOTOR_A_COUNTERCLOCKWISE_01_STOP; // Changed state name
      }
      break;
    case p2MOTOR_A_COUNTERCLOCKWISE_01_STOP: // New state name
      if (currentMillis - previousMillis >= 1500) {
        analogWrite(ena, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = p2MOTOR_B_CLOCKWISE_01;
      }
      break;
    case p2MOTOR_B_CLOCKWISE_01:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_01_STOP;
    }
    break;
  case p2MOTOR_B_CLOCKWISE_01_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_C_RESET_01;
    }
    break;
  case p2MOTOR_C_RESET_01:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorC.write(0);
      previousMillis = currentMillis;
      currentState = p2MOTOR_D_ROTATE_01;
      delay(1000);
    }
    break;
  case p2MOTOR_D_ROTATE_01:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorD.write(180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_C_ROTATE_01;
      delay(1000);
    }
    break;
  case p2MOTOR_C_ROTATE_01:
    if (currentMillis - previousMillis >= servoMoveDuration) {
      motorC.write(180);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_01;
      delay(1000);
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_01:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_01_STOP;
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_01_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_01;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_01:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(ena, 235);
      previousMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_01_STOP;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_01_STOP:
    if (currentMillis - previousMillis >= 420) {
      analogWrite(ena, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_02;
    }
    break;

  // MOTOR_B_CLOCKWISE_02
  case p2MOTOR_B_CLOCKWISE_02:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_02_STOP;
    }
    break;
  case p2MOTOR_B_CLOCKWISE_02_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_C_RESET_02;
    }
    break;
  case p2MOTOR_C_RESET_02:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorC.write(0);
      previousMillis = currentMillis;
      currentState = p2MOTOR_D_RESET_02;
      delay(1000);
    }
    break;
  case p2MOTOR_D_RESET_02:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorD.write(0);
      previousMillis = currentMillis;
      currentState = p2MOTOR_C_ROTATE_02;
      delay(1000);
    }
    break;
  case p2MOTOR_C_ROTATE_02:
    if (currentMillis - previousMillis >= servoMoveDuration) {
      motorC.write(180);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_02;
      delay(1000);
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_02:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_02_STOP;
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_02_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_02;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_02:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(ena, 235);
      previousMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_02_STOP;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_02_STOP:
    if (currentMillis - previousMillis >= 420) {
      analogWrite(ena, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_03;
    }
    break;

  // MOTOR_B_CLOCKWISE_03
  case p2MOTOR_B_CLOCKWISE_03:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_03_STOP;
    }
    break;
  case p2MOTOR_B_CLOCKWISE_03_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_C_RESET_03;
    }
    break;
  case p2MOTOR_C_RESET_03:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorC.write(0);
      previousMillis = currentMillis;
      currentState = p2MOTOR_D_ROTATE_03;
      delay(1000);
    }
    break;
  case p2MOTOR_D_ROTATE_03:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorD.write(180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_C_ROTATE_03;
      delay(1000);
    }
    break;
  case p2MOTOR_C_ROTATE_03:
    if (currentMillis - previousMillis >= servoMoveDuration) {
      motorC.write(180);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_03;
      delay(1000);
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_03:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_03_STOP;
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_03_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_03;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_03:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(ena, 235);
      previousMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_03_STOP;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_03_STOP:
    if (currentMillis - previousMillis >= 420) {
      analogWrite(ena, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_04;
    }
    break;

  // MOTOR_B_CLOCKWISE_04
  case p2MOTOR_B_CLOCKWISE_04:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_04_STOP;
    }
    break;
  case p2MOTOR_B_CLOCKWISE_04_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_C_RESET_04;
    }
    break;
  case p2MOTOR_C_RESET_04:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorC.write(0);
      previousMillis = currentMillis;
      currentState = p2MOTOR_D_RESET_04;
      delay(1000);
    }
    break;
  case p2MOTOR_D_RESET_04:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorD.write(0);
      previousMillis = currentMillis;
      currentState = p2MOTOR_C_ROTATE_04;
      delay(1000);
    }
    break;
  case p2MOTOR_C_ROTATE_04:
    if (currentMillis - previousMillis >= servoMoveDuration) {
      motorC.write(180);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_04;
      delay(1000);
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_04:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_04_STOP;
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_04_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_04;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_04:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(ena, 235);
      previousMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_04_STOP;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_04_STOP:
    if (currentMillis - previousMillis >= 420) {
      analogWrite(ena, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_05;
    }
    break;

  // MOTOR_B_CLOCKWISE_05
  case p2MOTOR_B_CLOCKWISE_05:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_05_STOP;
    }
    break;
  case p2MOTOR_B_CLOCKWISE_05_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_C_RESET_05;
    }
    break;
  case p2MOTOR_C_RESET_05:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorC.write(0);
      previousMillis = currentMillis;
      currentState = p2MOTOR_D_ROTATE_05;
      delay(1000);
    }
    break;
  case p2MOTOR_D_ROTATE_05:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorD.write(180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_C_ROTATE_05;
      delay(1000);
    }
    break;
  case p2MOTOR_C_ROTATE_05:
    if (currentMillis - previousMillis >= servoMoveDuration) {
      motorC.write(180);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_05;
      delay(1000);
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_05:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_05_STOP;
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_05_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_A_COUNTERCLOCKWISE_05;
    }
    break;
  case p2MOTOR_A_COUNTERCLOCKWISE_05:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(ena, 235);
      previousMillis = currentMillis;
      currentState = p2MOTOR_A_COUNTERCLOCKWISE_05_STOP;
    }
    break;
  case p2MOTOR_A_COUNTERCLOCKWISE_05_STOP:
    if (currentMillis - previousMillis >= 420) {
      analogWrite(ena, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_06;
    }
    break;

  // MOTOR_B_CLOCKWISE_06
  case p2MOTOR_B_CLOCKWISE_06:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_06_STOP;
    }
    break;
  case p2MOTOR_B_CLOCKWISE_06_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_C_RESET_06;
    }
    break;
  case p2MOTOR_C_RESET_06:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorC.write(0);
      previousMillis = currentMillis;
      currentState = p2MOTOR_D_RESET_06;
      delay(1000);
    }
    break;
  case p2MOTOR_D_RESET_06:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorD.write(0);
      previousMillis = currentMillis;
      currentState = p2MOTOR_C_ROTATE_06;
      delay(1000);
    }
    break;
  case p2MOTOR_C_ROTATE_06:
    if (currentMillis - previousMillis >= servoMoveDuration) {
      motorC.write(180);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_06;
      delay(1000);
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_06:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_06_STOP;
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_06_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_06;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_06:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(ena, 235);
      previousMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_06_STOP;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_06_STOP:
    if (currentMillis - previousMillis >= 420) {
      analogWrite(ena, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_07;
    }
    break;

    // MOTOR_B_CLOCKWISE_07
  case p2MOTOR_B_CLOCKWISE_07:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_07_STOP;
    }
    break;
  case p2MOTOR_B_CLOCKWISE_07_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_C_RESET_07;
    }
    break;
  case p2MOTOR_C_RESET_07:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorC.write(0);
      previousMillis = currentMillis;
      currentState = p2MOTOR_D_ROTATE_07;
      delay(1000);
    }
    break;
  case p2MOTOR_D_ROTATE_07:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorD.write(180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_C_ROTATE_07;
      delay(1000);
    }
    break;
  case p2MOTOR_C_ROTATE_07:
    if (currentMillis - previousMillis >= servoMoveDuration) {
      motorC.write(180);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_07;
      delay(1000);
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_07:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_07_STOP;
    }
    break;
   case p2MOTOR_B_COUNTERCLOCKWISE_07_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_07;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_07:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(ena, 235);
      previousMillis = currentMillis;
      currentState = p2MOTOR_A_CLOCKWISE_07_STOP;
    }
    break;
  case p2MOTOR_A_CLOCKWISE_07_STOP:
    if (currentMillis - previousMillis >= 420) {
      analogWrite(ena, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_08;
    }
    break;

  // MOTOR_B_CLOCKWISE_08
  case p2MOTOR_B_CLOCKWISE_08:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_08_STOP;
    }
    break;
  case p2MOTOR_B_CLOCKWISE_08_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_C_RESET_08;
    }
    break;
  case p2MOTOR_C_RESET_08:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorC.write(0);
      previousMillis = currentMillis;
      currentState =  p2MOTOR_D_RESET_08;
      delay(1000);
    }
    break;
  case p2MOTOR_D_RESET_08:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      motorD.write(0);
      previousMillis = currentMillis;
      currentState = p2MOTOR_C_ROTATE_08;
      delay(1000);
    }
    break;
  case p2MOTOR_C_ROTATE_08:
    if (currentMillis - previousMillis >= servoMoveDuration) {
      motorC.write(180);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_08;
      delay(1000);
    }
    break;
  case p2MOTOR_B_COUNTERCLOCKWISE_08:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enb, 180);
      previousMillis = currentMillis;
      currentState = p2MOTOR_B_COUNTERCLOCKWISE_08_STOP;
    }
    break;
   case p2MOTOR_B_COUNTERCLOCKWISE_08_STOP:
    if (currentMillis - previousMillis >= motorBRotationDuration) {
      analogWrite(enb, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_A_COUNTERCLOCKWISE_08;
    }
    break;
  case p2MOTOR_A_COUNTERCLOCKWISE_08:
    if (currentMillis - stateChangeMillis >= stateTransitionGap) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(ena, 235);
      previousMillis = currentMillis;
      currentState = p2MOTOR_A_COUNTERCLOCKWISE_08_STOP;
    }
    break;
  case p2MOTOR_A_COUNTERCLOCKWISE_08_STOP:
    if (currentMillis - previousMillis >= 420) {
      analogWrite(ena, 0);
      previousMillis = currentMillis;
      stateChangeMillis = currentMillis;
      currentState = p2MOTOR_B_CLOCKWISE_09;
    }
    break;
  }
}