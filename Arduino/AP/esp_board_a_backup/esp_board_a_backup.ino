#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <DallasTemperature.h>
#include <Adafruit_VL53L0X.h>
#include <esp_system.h>

// ESP32---------------------------------------------------------
// MAC address of the reciever ESP32 - pin38
uint8_t espBoardB_address[] = { 0xB0, 0xB2, 0x1C, 0x97, 0x6D, 0xB4 };

typedef struct struct_message {
  int angleX, angleY, angleZ;  // Gyroscope data
  bool gyroscopeDataSent;      // Flag to indicate if gyroscope data was sent
  bool gyroscopeEnterPressed;
  bool chooseOptionsLoaded;
  bool BIsPressed;
  bool outputCheckCalled;
  bool ultrasonicObjectDetected;
  bool waterFilled;  // water part completion condition
  bool motorPartCompleted;
  bool warningCalled, buzzerCalled, redLEDCalled, yellowLEDCalled, greenLEDCalled;
  String warningMessage;
  bool resetCalled;
  bool callRED;
  bool liquidUltrasonicCalled;
} struct_message;

struct_message myData;

// LCD and Keypad Initialization
LiquidCrystal_I2C lcd(0x27, 16, 2);

const byte ROWS = 4;
const byte COLS = 4;
char hexaKeys[ROWS][COLS] = {
  { 'D', '#', '0', '*' },
  { 'C', '9', '8', '7' },
  { 'B', '6', '5', '4' },
  { 'A', '3', '2', '1' }
};
byte rowPins[ROWS] = { 12, 13, 14, 27 };
byte colPins[COLS] = { 26, 25, 33, 32 };
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// Custom Characters
byte degreeChar[8] = { 0b00111, 0b00101, 0b00111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };
byte superscriptMinus3[8] = { 0b00111, 0b00001, 0b11011, 0b00001, 0b00111, 0b00000, 0b00000, 0b00000 };
byte upArrow[8] = { 0b00100, 0b01110, 0b11111, 0b11111, 0b01110, 0b01110, 0b01110, 0b01110 };
byte downArrow[8] = { 0b01110, 0b01110, 0b01110, 0b01110, 0b11111, 0b11111, 0b01110, 0b00100 };
byte barGraphChar[8] = { 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111 };

// TEMPERATURE SENSOR-------------------------------------------
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Input Output Variables----------------------------------------
bool completed = false, warningIndication = false;
String inputCValue = "", outputCValue = "";
bool enteringFirstInput = true, inputCHasDecimal = false, outputCHasDecimal = false, finalOutputIsDisplayed = false;
float inputC = 0.0, outputC = 0.0, temp = 0.0;
double outputV = 0.0;
char outputVStr[10];

bool gyroscopeEnterPressed = false, tofThresholdDetected = false, maxWaterLevelReached = true, fillWaterEnterPressed = false, BIsPressed = false, resetCalled = false, mixSecondStage = false, tempRunning = false;
// ULTRASONIC Variables-------------------------------------
const int trigPinUS = 16, echoPinUS = 17;
long duration;
float distance;

// Water Variables-------------------------------------------
const float minWaterLevel = 29.5, maxWaterLevel = 9.0;
#define PUMP_RELAY_PIN 23
#define VALVE_RELAY_PIN 19
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
// Refractive index of water
const float refractiveIndex = 1.333;
bool waterFilled = false;

// Enumeration for states
enum State {
  GYROSCOPE_STATE,
  INPUT_C_STATE,
  INPUT_V_STATE,
  OUTPUT_C_STATE,
  CHOOSE_DILUTE_OR_MIX_STATE,
  FILL_WATER_STATE,
  HANDLE_DILUTE_OPTION,
  HANDLE_MIX_OPTION,
  CHECK_ULTRASONIC_SENSOR,
  MIX_PLACE_BEAKERS_STATE
  // Add more states as needed
};
State currentState = GYROSCOPE_STATE;

// Function Declarations
void initializeComponents();
void getPressedKey();
void handleHashKey();
void handleAsteriskKey();
void handleAKey();
void handleBKey();
void handleCKey();
void resetCircuit();
void handleNumberKey(char key);
void clearInputCValue();
void clearOutputCValue();
void gyroscopeToDisplay();
void chooseDiluteOrMix();
void mixToDisplay();
void ultrasonicToDisplay();
float tofMeasureWaterLevel();
void runMeasureSendWaterVolume();
void fillWater();
void displayInitialMessage(String topRowMessage, String bottomRowMessage);
void displayOutputV();
void runTemp();
void runUltrasonic();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len);
void sendData();
void warning(const char *message);

void setup() {
  Serial.begin(115200);
  initializeComponents();
}

void loop() {
  getPressedKey();

  if (finalOutputIsDisplayed && !myData.motorPartCompleted) {
    // Serial.println("Final output was displayed");
    myData.outputCheckCalled = true;
    if (!waterFilled) {
      ultrasonicToDisplay();
    } else {
      runTemp();
      tempRunning = true;  // Set flag when temperature function starts
    }
  }
  if (myData.ultrasonicObjectDetected && !waterFilled) {
    Serial.println("Output beaker detected");
    runMeasureSendWaterVolume();
  }
  if (myData.waterFilled && !myData.motorPartCompleted && myData.liquidUltrasonicCalled) {
    Serial.println("TOF threshold detected");
    runUltrasonic();
  }
  // if (BIsPressed && myData.ultrasonicObjectDetected && !myData.motorPartCompleted) {
  //   runUltrasonic();
  //   delay(1000);
  //   runTemp();
  //   delay(1000);
  //   mixSecondStage = true;
  //   mixToDisplay();
  // }

  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime > 1000) {
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

  // LCD Initialization
  lcd.init();
  lcd.backlight();
  lcd.clear();  // Clear the display
  delay(500);
  lcd.createChar(0, degreeChar);
  lcd.createChar(1, superscriptMinus3);
  lcd.createChar(2, upArrow);
  lcd.createChar(3, downArrow);
  lcd.createChar(4, barGraphChar);

  // Water Pump & Solenoid Valve Initialization
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(VALVE_RELAY_PIN, OUTPUT);
  digitalWrite(PUMP_RELAY_PIN, HIGH);
  digitalWrite(VALVE_RELAY_PIN, LOW);

  // TOF Sensor Initialization
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  Serial.println(F("VL53L0X started"));

  // Ultrasonic Sensor Initialization
  pinMode(trigPinUS, OUTPUT);
  pinMode(echoPinUS, INPUT);
}

void getPressedKey() {
  char customKey = customKeypad.getKey();
  if (!customKey) return;
  Serial.print("Key Pressed: ");
  Serial.println(customKey);

  switch (customKey) {
    case '#': handleHashKey(); break;
    case '*': handleAsteriskKey(); break;
    // case 'A': handleAKey(); break;
    // case 'B': handleBKey(); break;
    case 'C': handleCKey(); break;
    case 'D': resetCircuit(); break;
    default: handleNumberKey(customKey); break;
  }
}

void handleHashKey() {
  Serial.print("Current State: ");
  Serial.println(currentState);

  switch (currentState) {
    case GYROSCOPE_STATE:
      if (!gyroscopeEnterPressed) {
        Serial.println("Entering gyroscope mode...");
        // chooseDiluteOrMix();
        gyroscopeEnterPressed = true;
        myData.gyroscopeEnterPressed = gyroscopeEnterPressed;
        fillWater();
        currentState = FILL_WATER_STATE;
        Serial.println("Gyroscope Enter Pressed Set to True");
      }
      break;

      // case CHOOSE_DILUTE_OR_MIX_STATE:
      //   // Wait for user to press A or B (handled in handleAKey and handleBKey functions)
      //   Serial.println("Option Selected");
      //   break;

    case FILL_WATER_STATE:
      // Check if the water level has reached the maximum
      if (maxWaterLevelReached) {
        fillWaterEnterPressed = true;
        currentState = INPUT_C_STATE;
        // displayInitialMessage("Input C = ", "moldm");
        enteringFirstInput = true;
      }
      break;

    case INPUT_C_STATE:
      // Process user input for Input C
      inputC = inputCValue.toFloat();
      Serial.print("Input C = ");
      Serial.print(inputC);
      Serial.println(" moldm-3");
      lcd.clear();
      displayInitialMessage("Output C = ", "moldm");
      enteringFirstInput = false;
      currentState = OUTPUT_C_STATE;
      break;

    case OUTPUT_C_STATE:
      // Process user input for Output C
      if (!enteringFirstInput) {
        outputC = outputCValue.toFloat();
        Serial.print("Output C = ");
        Serial.print(outputC);
        Serial.println(" moldm-3");
        lcd.clear();
        displayOutputV();
        currentState = CHECK_ULTRASONIC_SENSOR;
      }
      break;

      // case CHECK_ULTRASONIC_SENSOR:
      // Activate ultrasonic sensor and check for object within the specified threshold

      break;

      // case MIX_PLACE_BEAKERS_STATE:
      // if (BIsPressed || mixSecondStage) {
      //   myData.outputCheckCalled = true;
      //   currentState = CHECK_ULTRASONIC_SENSOR;
      // }

      break;

    // Add more cases for additional states if needed
    default:
      break;
  }
}

void handleAsteriskKey() {
  if (inputCValue.length() > 0 && outputCValue.length() == 0) {
    clearInputCValue();
  } else if (inputCValue.length() > 0 && outputCValue.length() > 0) {
    clearOutputCValue();
  }
}

// void handleAKey() {
//   if (gyroscopeEnterPressed && !finalOutputIsDisplayed) {
//     fillWater();
//     currentState = FILL_WATER_STATE;
//   }
// }

// void handleBKey() {
//   if (gyroscopeEnterPressed && !finalOutputIsDisplayed) {
//     BIsPressed = true;
//     myData.BIsPressed = BIsPressed;
//     mixToDisplay();
//     currentState = HANDLE_MIX_OPTION;
//   }
// }

void handleCKey() {
  if (enteringFirstInput) {
    if (!inputCHasDecimal) {
      inputCValue += ".";
      inputCHasDecimal = true;
      lcd.setCursor(10 + inputCValue.length() - 1, 0);
      lcd.print(".");
    }
  } else {
    if (!outputCHasDecimal) {
      outputCValue += ".";
      outputCHasDecimal = true;
      lcd.setCursor(11 + outputCValue.length() - 1, 0);
      lcd.print(".");
    }
  }
}

void resetCircuit() {
  resetCalled = true;
  myData.resetCalled = resetCalled;
  sendData();
  Serial.println("ESP32 will reset in 3 seconds...");
  delay(3000);
  esp_restart();
}

void handleNumberKey(char key) {
  if (enteringFirstInput) {
    inputCValue += key;
    lcd.setCursor(10, 0);
    lcd.print(inputCValue);
  } else {
    outputCValue += key;
    lcd.setCursor(11, 0);
    lcd.print(outputCValue);
  }
}

void clearInputCValue() {
  inputCValue = "";
  lcd.setCursor(10, 0);
  lcd.print("          ");
  lcd.setCursor(10, 0);
}

void clearOutputCValue() {
  outputCValue = "";
  lcd.setCursor(11, 0);
  lcd.print("          ");
  lcd.setCursor(11, 0);
}

void gyroscopeToDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gyroscope(X,Y,Z)");
  lcd.setCursor(0, 1);
  lcd.print(myData.angleX);
  lcd.setCursor(4, 1);
  lcd.print(",");
  lcd.setCursor(5, 1);
  lcd.print(myData.angleY);
  lcd.setCursor(9, 1);
  lcd.print(",");
  lcd.setCursor(10, 1);
  lcd.print(myData.angleZ);
}

// void chooseDiluteOrMix() {
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print("A - Dilute");
//   lcd.setCursor(0, 1);
//   lcd.print("B - Mix");
//   myData.chooseOptionsLoaded = true;
// }

// void mixToDisplay() {
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print("Place liquids");
//   lcd.setCursor(0, 1);
//   lcd.print("in the beakers");

//   myData.outputCheckCalled = false;
//   myData.motorPartCompleted = false;
// }

void ultrasonicToDisplay() {
  lcd.setCursor(0, 1);
  // lcd.print("                ");
  lcd.print("Beaker detected");
}

float tofMeasureWaterLevel() {
  const int numSamples = 30;
  float totalDistance = 0;
  float averageDistance = 0;
  int validSamples = 0;

  for (int i = 0; i < numSamples; i++) {
    VL53L0X_RangingMeasurementData_t measure;

    // Perform the measurement
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {                          // phase failures have incorrect data
      float distance = measure.RangeMilliMeter / 10.0;       // Convert to cm
      float correctedDistance = distance * refractiveIndex;  // Apply refraction correction
      totalDistance += correctedDistance;
      validSamples++;
    }

    delay(10);  // Delay to achieve approximately 100 samples per second
  }

  if (validSamples > 0) {
    averageDistance = totalDistance / validSamples;
    Serial.print("Average corrected distance: ");
    Serial.print(averageDistance);
    Serial.println(" cm");
  } else {
    Serial.println("No valid samples");
  }

  return averageDistance;

  delay(1000 - (numSamples * 10));  // Adjust delay to complete one second cycle
}

void runMeasureSendWaterVolume() {
  float currentWaterLevel = tofMeasureWaterLevel();
  float currentVolume = (currentWaterLevel - maxWaterLevel) * 1.694915;
  float pumpRate = 13;  // Pump rate in ml/s

  Serial.print("Current Water Level (mm): ");
  Serial.println(currentWaterLevel);
  Serial.print("Current Water Volume (ml): ");
  Serial.println(currentVolume);

  if (currentWaterLevel >= minWaterLevel) {
    Serial.println("Water volume is not enough");
    warningIndication = true;
    warning("Low Water Level");
    digitalWrite(PUMP_RELAY_PIN, HIGH);
    digitalWrite(VALVE_RELAY_PIN, HIGH);
    while (1)
      ;
  }
  // if (currentVolume < (outputV - 2)) {
  //   Serial.println("Filling water...");
  //   digitalWrite(PUMP_RELAY_PIN, LOW);
  //   digitalWrite(VALVE_RELAY_PIN, LOW);
  // } else {
  //   Serial.println("Threshold reached, stopping pump and closing valve");
  //   digitalWrite(PUMP_RELAY_PIN, HIGH);
  //   digitalWrite(VALVE_RELAY_PIN, HIGH);
  //   myData.tofThresholdDetected = true;
  // }
  if (outputV > 0 && waterFilled == false) {
    int pumpTime = (outputV - 4) / pumpRate;  // Calculate the time in seconds to pump the desired volume

    // Activate water pump and solenoid valve
    digitalWrite(PUMP_RELAY_PIN, LOW);
    digitalWrite(VALVE_RELAY_PIN, HIGH);

    // Wait for the calculated time
    delay(pumpTime * 1000);  // Convert seconds to milliseconds

    // Deactivate water pump and close solenoid valve
    digitalWrite(PUMP_RELAY_PIN, HIGH);
    digitalWrite(VALVE_RELAY_PIN, LOW);

    waterFilled = true;
    myData.waterFilled = waterFilled;
    Serial.println("Threshold reached from pump, stopping pump and closing valve");
    sendData();
  }
  delay(100);
}

void fillWater() {
  lcd.clear();
  lcd.print("Fill all Liquids");

  unsigned long startTime = millis();  // Start time for timeout logic
  float currentWaterLevel = tofMeasureWaterLevel();
  int currentVolume = 0;

  while (true) {
    currentWaterLevel = tofMeasureWaterLevel();
    currentVolume = (minWaterLevel - currentWaterLevel) * 1.694915;

    if (currentWaterLevel == -1) {
      lcd.clear();
      lcd.print("Error: Out of range");
      Serial.println("Error: ToF sensor out of range");
      warningIndication = true;
      warning("Error: ToF sensor");
      break;
    }

    int fillAmount = ((minWaterLevel - currentWaterLevel) / (minWaterLevel - maxWaterLevel)) * 16;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Water Level:");
    for (int i = 0; i <= fillAmount; i++) {
      lcd.setCursor(i, 1);
      lcd.write(byte(4));
    }
    delay(2000);

    if (currentWaterLevel <= maxWaterLevel) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Max Water Level");
      lcd.setCursor(0, 1);
      lcd.print("Reached!");
      Serial.println("Max water level reached.");
      maxWaterLevelReached = true;
      delay(5000);

      if (!waterFilled) {
        // Check for timeout (e.g., 2 minutes)
        if (millis() - startTime > 300000) {  // 300000 ms = 5 minutes
          Serial.println("Timeout: Max water level not reached");
          warningIndication = true;
          warning("Low Water Level");
          fillWater();
          break;
        }
      }

      // Transition to the INPUT_C_STATE
      // currentState = INPUT_C_STATE;
      displayInitialMessage("Input C = ", "moldm");
      break;
    }
  }
}

void displayInitialMessage(String topRowMessage, String bottomRowMessage) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(topRowMessage);
  if (bottomRowMessage.length() > 0) {
    lcd.setCursor(0, 1);
    lcd.print("(");
    lcd.print(bottomRowMessage);
    lcd.write(byte(1));
    lcd.print(")");
  }
}

void displayOutputV() {
  lcd.clear();
  outputV = (inputC * 4) / outputC;
  dtostrf(outputV, 9, 5, outputVStr);
  lcd.setCursor(0, 0);
  lcd.print("Vo = ");
  lcd.print(outputVStr);
  lcd.print("ml");
  Serial.print("Output V = ");
  Serial.print(outputV);
  Serial.println("ml");

  if (outputV > 225 || outputV < 50) {
    warningIndication = true;
    warning("Vo Out of Range");
    myData.callRED = true;
    sendData();
    delay(5000);
    myData.resetCalled = true;
    resetCircuit();
  }

  if (myData.ultrasonicObjectDetected) {
    Serial.println("Ultrasonic beaker check complete");
    ultrasonicToDisplay();
  }

  finalOutputIsDisplayed = true;
}

void runTemp() {
  float previousTemp = temp;
  sensors.requestTemperatures();
  temp = sensors.getTempCByIndex(0);
  lcd.setCursor(0, 1);
  lcd.print("                ");
  if (temp != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");
    lcd.setCursor(0, 1);
    lcd.print("Temp = ");
    lcd.print(temp);
    lcd.write(byte(0));  // custom degree symbol
    lcd.print("C");
  } else {
    Serial.println("Error: Could not read temperature data");
    lcd.setCursor(0, 1);
    lcd.print("Temp: No Device");
  }
  float tempDifference = temp - previousTemp;
  if (tempDifference > 0) {
    lcd.setCursor(15, 1);
    lcd.write(byte(2));
  } else if (tempDifference < 0) {
    lcd.setCursor(15, 1);
    lcd.write(byte(3));
  }

  if (completed) {
    Serial.println("Temperature function stopped due to success.");
    return;  // Exit the function if success is called
  }
  if (warningIndication) {
    Serial.println("Temperature function stopped due to Warning.");
    return;  // Exit the function if warning is called
  }

  delay(100);
}

void runUltrasonic() {
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
  if (distance > 6.5) {
    warningIndication = true;
    myData.redLEDCalled = true;
    warning("No Liquid Detected");
  }
  delay(1000);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\nLast Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);

  if (gyroscopeEnterPressed) {
    Serial.println("Gyroscope Finished");
  } else {
    gyroscopeToDisplay();
  }

  if (myData.gyroscopeDataSent) {
    // Handle gyroscope data
    Serial.print("Received Gyroscope Data - X: ");
    Serial.print(myData.angleX);
    Serial.print(", Y: ");
    Serial.print(myData.angleY);
    Serial.print(", Z: ");
    Serial.println(myData.angleZ);
  }

  if (myData.motorPartCompleted) {
    completed = true;
    success();  // Call success function if motor part is completed
  }

  if (myData.warningCalled) {
    warningIndication = true;
    warning(myData.warningMessage.c_str());  // Call warning function if warning flag is set
  }
  if (myData.resetCalled) {
    Serial.println("RESET called");
    resetCircuit();
  }
}

void sendData() {
  esp_err_t result = esp_now_send(espBoardB_address, (uint8_t *)&myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

void warning(const char *message) {
  // Set the warning flag and message
  myData.warningCalled = true;
  myData.warningMessage = message;

  // Send the data to the other board
  sendData();

  // Display warning message on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Warning:");
  lcd.setCursor(0, 1);
  lcd.print(message);

  // Send warning message to the other ESP32 board
  struct_message warningData = myData;
  esp_now_send(espBoardB_address, (uint8_t *)&warningData, sizeof(warningData));

  // Keep the warning state for 5 seconds
  delay(5000);

  // turn off the warning flag
  myData.warningCalled = false;

  // Reset the tempRunning flag
  tempRunning = false;
  // warningIndication = false;

  delay(5000);
  myData.resetCalled = true;
  resetCircuit();
}

void success() {
  // Display warning message on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Success! Press");
  lcd.setCursor(0, 1);
  lcd.print("Reset to redo");

  // Keep the success state for 5 seconds
  delay(5000);

  // Reset the tempRunning flag
  tempRunning = false;
}