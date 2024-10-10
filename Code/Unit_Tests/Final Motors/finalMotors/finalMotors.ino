#include <ESP32Servo.h>

// Define motor control pins for Motor A and B (connected to the first motor driver)
int ena = 19;   // Motor A enable pin
int in1 = 18;    // Motor A input 1
int in2 = 5;    // Motor A input 2
int in3 = 4;    // Motor B input 1
int in4 = 2;   // Motor B input 2
int enb = 15;   // Motor B enable pin

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
const unsigned long motorBRotationDuration = 240;  // Duration of each Motor B rotation in milliseconds
const unsigned long motorBStopDuration = 1000;     // Stop duration after each Motor B rotation in milliseconds

// Timing constants for Servos
const unsigned long servoMoveDuration = 1000;      // Duration for servo movements

// Timing constant for state transition gap
const unsigned long stateTransitionGap = 1000; // 1500 milliseconds gap between state transitions

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

MotorState currentState = MOTOR_A_COUNTERCLOCKWISE_01; // Initialize the state machine with the first state

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
       case MOTOR_A_COUNTERCLOCKWISE_01: // New state name
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in1, LOW); // Set motor A to rotate counterclockwise
        digitalWrite(in2, HIGH); // Set motor A to rotate counterclockwise
        analogWrite(ena, 245);
        previousMillis = currentMillis;
        currentState = MOTOR_A_COUNTERCLOCKWISE_01_STOP; // Changed state name
      }
      break;
    case MOTOR_A_COUNTERCLOCKWISE_01_STOP: // New state name
      if (currentMillis - previousMillis >= 1500) {
        analogWrite(ena, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_01;
      }
      break;
    case MOTOR_B_CLOCKWISE_01:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_01_STOP;
      }
      break;
    case MOTOR_B_CLOCKWISE_01_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_C_RESET_01;
      }
      break;
    case MOTOR_C_RESET_01:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        motorC.write(0);
        previousMillis = currentMillis;
        currentState = MOTOR_D_ROTATET_01;
        delay(1000);
      }
      break;
    case MOTOR_D_ROTATET_01:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        motorD.write(180);
        previousMillis = currentMillis;
        currentState = MOTOR_C_ROTATET_01;
        delay(1000);
      }
      break;
    case MOTOR_C_ROTATET_01:
      if (currentMillis - previousMillis >= servoMoveDuration) {
        motorC.write(180);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_01;
        delay(1000);
      }
      break;
    case MOTOR_B_COUNTERCLOCKWISE_01:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_01_STOP;
      }
      break;
    case MOTOR_B_COUNTERCLOCKWISE_01_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_A_CLOCKWISE_01;
      }
      break;
    case MOTOR_A_CLOCKWISE_01:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(ena, 235);
        previousMillis = currentMillis;
        currentState = MOTOR_A_CLOCKWISE_01_STOP;
      }
      break;
    case MOTOR_A_CLOCKWISE_01_STOP:
      if (currentMillis - previousMillis >= 420) {
        analogWrite(ena, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_02;
      }
      break;
//-----------------------------------------------------------------------
    // Add similar cases for other states
    case MOTOR_B_CLOCKWISE_02:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_02_STOP;
      }
      break;
    case MOTOR_B_CLOCKWISE_02_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_C_RESET_02;
      }
      break;
    case MOTOR_C_RESET_02:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        motorC.write(0);
        previousMillis = currentMillis;
        currentState = MOTOR_D_RESET_02;
        delay(1000);
      }
      break;
    case MOTOR_D_RESET_02:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        motorD.write(0);
        previousMillis = currentMillis;
        currentState = MOTOR_C_ROTATET_02;
        delay(1000);
      }
      break;
    case MOTOR_C_ROTATET_02:
      if (currentMillis - previousMillis >= servoMoveDuration) {
        motorC.write(180);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_02;
        delay(1000);
      }
      break;
    case MOTOR_B_COUNTERCLOCKWISE_02:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_02_STOP;
      }
      break;
    case MOTOR_B_COUNTERCLOCKWISE_02_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_A_CLOCKWISE_02;
      }
      break;
    case MOTOR_A_CLOCKWISE_02:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(ena, 245);
        previousMillis = currentMillis;
        currentState = MOTOR_A_CLOCKWISE_02_STOP;
      }
      break;
    case MOTOR_A_CLOCKWISE_02_STOP:
      if (currentMillis - previousMillis >= 2000) {
        analogWrite(ena, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_03;
      }
      break;
//-----------------------------------------------------------------------
      
    case MOTOR_B_CLOCKWISE_03:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_03_STOP;
      }
      break;
    case MOTOR_B_CLOCKWISE_03_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_C_RESET_03;
      }
      break;
    case MOTOR_C_RESET_03:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        motorC.write(0);
        previousMillis = currentMillis;
        currentState = MOTOR_D_ROTATET_03;
        delay(1000);
      }
      break;
    case MOTOR_D_ROTATET_03:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        motorD.write(180);
        previousMillis = currentMillis;
        currentState = MOTOR_C_ROTATET_03;
        delay(1000);
      }
      break;
    case MOTOR_C_ROTATET_03:
      if (currentMillis - previousMillis >= servoMoveDuration) {
        motorC.write(180);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_03;
        delay(1000);
      }
      break;
    case MOTOR_B_COUNTERCLOCKWISE_03:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_03_STOP;
      }
      break;
    case MOTOR_B_COUNTERCLOCKWISE_03_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_A_COUNTERCLOCKWISE_03; // Changed state name
      }
      break;
    case MOTOR_A_COUNTERCLOCKWISE_03: // New state name
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in1, LOW); // Set motor A to rotate counterclockwise
        digitalWrite(in2, HIGH); // Set motor A to rotate counterclockwise
        analogWrite(ena, 230);
        previousMillis = currentMillis;
        currentState = MOTOR_A_COUNTERCLOCKWISE_03_STOP; // Changed state name
      }
      break;
    case MOTOR_A_COUNTERCLOCKWISE_03_STOP: // New state name
      if (currentMillis - previousMillis >= 300) {
        analogWrite(ena, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_04;
      }
      break;

      //--------------------------------------------------------------------
        case MOTOR_B_CLOCKWISE_04:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_04_STOP;
      }
      break;
    case MOTOR_B_CLOCKWISE_04_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_D_RESET_04;
      }
      break;
    // case MOTOR_C_RESET_04:
    //   if (currentMillis - stateChangeMillis >= stateTransitionGap) {
    //     motorC.write(180);
    //     previousMillis = currentMillis;
    //     currentState = MOTOR_D_RESET_04;
    //     delay(1000);
    //   }
    //   break;
    case MOTOR_D_RESET_04:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        motorD.write(0);
        previousMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_04;
        delay(1000);
      }
      break;
    // case MOTOR_C_ROTATET_04:
    //   if (currentMillis - previousMillis >= servoMoveDuration) {
    //     motorC.write(180);
    //     previousMillis = currentMillis;
    //     stateChangeMillis = currentMillis;
    //     currentState = MOTOR_B_COUNTERCLOCKWISE_04;
    //     delay(1000);
    //   }
    //   break;
    case MOTOR_B_COUNTERCLOCKWISE_04:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_04_STOP;
      }
      break;
    case MOTOR_B_COUNTERCLOCKWISE_04_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_A_COUNTERCLOCKWISE_04; // Changed state name
      }
      break;
    case MOTOR_A_COUNTERCLOCKWISE_04: // New state name
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in1, LOW); // Set motor A to rotate counterclockwise
        digitalWrite(in2, HIGH); // Set motor A to rotate counterclockwise
        analogWrite(ena, 238);
        previousMillis = currentMillis;
        currentState = MOTOR_A_COUNTERCLOCKWISE_04_STOP; // Changed state name
      }
      break;
    case MOTOR_A_COUNTERCLOCKWISE_04_STOP: // New state name
      if (currentMillis - previousMillis >= 400) {
        analogWrite(ena, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_05;
      }
      break;

    //--------------------------------------------------------------------

    case MOTOR_B_CLOCKWISE_05:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_05_STOP;
      }
      break;
    case MOTOR_B_CLOCKWISE_05_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_C_RESET_05;
      }
      break;
    case MOTOR_C_RESET_05:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        motorC.write(45);
        previousMillis = currentMillis;
        currentState = MOTOR_D_ROTATET_05;
        delay(1000);
      }
      break;
    case MOTOR_D_ROTATET_05:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        motorD.write(180);
        previousMillis = currentMillis;
        currentState = MOTOR_C_ROTATET_05;
        delay(1000);
      }
      break;
    case MOTOR_C_ROTATET_05:
      if (currentMillis - previousMillis >= servoMoveDuration) {
        motorC.write(180);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_05;
        delay(1000);
      }
      break;
    case MOTOR_B_COUNTERCLOCKWISE_05:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_05_STOP;
      }
      break;
    case MOTOR_B_COUNTERCLOCKWISE_05_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_A_CLOCKWISE_05;
      }
      break;
    case MOTOR_A_CLOCKWISE_05:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(ena, 230);
        previousMillis = currentMillis;
        currentState = MOTOR_A_CLOCKWISE_05_STOP;
      }
      break;
    case MOTOR_A_CLOCKWISE_05_STOP:
      if (currentMillis - previousMillis >= 490) {
        analogWrite(ena, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_06;
      }
      break;

      //------------------------------------------------------------------

      case MOTOR_B_CLOCKWISE_06:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_06_STOP;
      }
      break;
    case MOTOR_B_CLOCKWISE_06_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_D_RESET_06;
      }
      break;
    // case MOTOR_C_RESET_06:
    //   if (currentMillis - stateChangeMillis >= stateTransitionGap) {
    //     motorC.write(180);
    //     previousMillis = currentMillis;
    //     currentState = MOTOR_D_RESET_06;
    //     delay(1000);
    //   }
    //   break;
    case MOTOR_D_RESET_06:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        motorD.write(0);
        previousMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_06;
        delay(1000);
      }
      break;
    // case MOTOR_C_ROTATET_06:
    //   if (currentMillis - previousMillis >= servoMoveDuration) {
    //     motorC.write(180);
    //     previousMillis = currentMillis;
    //     stateChangeMillis = currentMillis;
    //     currentState = MOTOR_B_COUNTERCLOCKWISE_06;
    //     delay(1000);
    //   }
    //   break;
    case MOTOR_B_COUNTERCLOCKWISE_06:
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enb, 180);
        previousMillis = currentMillis;
        currentState = MOTOR_B_COUNTERCLOCKWISE_06_STOP;
      }
      break;
    case MOTOR_B_COUNTERCLOCKWISE_06_STOP:
      if (currentMillis - previousMillis >= motorBRotationDuration) {
        analogWrite(enb, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_A_COUNTERCLOCKWISE_06; // Changed state name
      }
      break;
    case MOTOR_A_COUNTERCLOCKWISE_06: // New state name
      if (currentMillis - stateChangeMillis >= stateTransitionGap) {
        digitalWrite(in1, LOW); // Set motor A to rotate counterclockwise
        digitalWrite(in2, HIGH); // Set motor A to rotate counterclockwise
        analogWrite(ena, 230);
        previousMillis = currentMillis;
        currentState = MOTOR_A_COUNTERCLOCKWISE_06_STOP; // Changed state name
      }
      break;
    case MOTOR_A_COUNTERCLOCKWISE_06_STOP: // New state name
      if (currentMillis - previousMillis >= 2000) {
        analogWrite(ena, 0);
        previousMillis = currentMillis;
        stateChangeMillis = currentMillis;
        currentState = MOTOR_B_CLOCKWISE_07;
      }
      break;
  }
}
