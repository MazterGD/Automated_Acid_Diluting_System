#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <DallasTemperature.h>
#include <Adafruit_VL53L0X.h>

// ESP32---------------------------------------------------------
// MAC address of the receiver ESP32 - pin38
uint8_t espBoardB_address[] = { 0xB0, 0xB2, 0x1C, 0x97, 0x6D, 0xB4 };

// Structure to hold received data
typedef struct struct_message {
  int angleX, angleY, angleZ;  // Gyroscope data
  bool finalOutputIsDisplayed;
  bool ultrasonicObjectDetected;
  bool tofThresholdDetected;  // water part completion condition
} struct_message;

struct_message myData;

// Initialize the LCD with I2C address 0x27 and screen size of 16x2
LiquidCrystal_I2C lcd(0x27, 16, 2);

// KEYPAD-------------------------------------------------
// Initialize the keypad
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

// Create a custom character array for the degree symbol
byte degreeChar[8] = {
  0b00111,
  0b00101,
  0b00111,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

// Create a custom character array for superscript minus 3 symbol
byte superscriptMinus3[8] = {
  0b00111,
  0b00001,
  0b11011,
  0b00001,
  0b00111,
  0b00000,
  0b00000,
  0b00000
};

// Create a custom character array for the up arrow symbol
byte upArrow[8] = {
  0b00100,
  0b01110,
  0b11111,
  0b11111,
  0b01110,
  0b01110,
  0b01110,
  0b01110
};
// Create a custom character array for the down arrow symbol
byte downArrow[8] = {
  0b01110,
  0b01110,
  0b01110,
  0b01110,
  0b11111,
  0b11111,
  0b01110,
  0b00100
};

// Create a custom character array for the filling bar graph symbol
byte fillBar[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

//TEMPERATURE SENSOR-------------------------------------------
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Input Output Variabes----------------------------------------
String inputCValue = "";
String outputCValue = "";
bool enteringFirstInput = true;
bool inputCHasDecimal = false;
bool outputCHasDecimal = false;
bool finalOutputIsDisplayed = false;
float inputC = 0.0;
float outputC = 0.0;
double outputV = 0.0;
float temp = 0.0;
char outputVStr[10];  // Buffer to hold the formatted outputV

bool gyroscopeEnterPressed = false;
bool tofThresholdDetected = false;
bool fillWaterEnterPressed = false;

// ULTRASONIC Variables-------------------------------------
const int trigPinUS = 17;
const int echoPinUS = 16;

// Variables to store the duration of the pulse and the distance
long duration;
float distance;

// Water Variables-------------------------------------------
const float waterBeakerHeight = 167.0;
const float maxWaterLevel = 50.0;

#define PUMP_RELAY_PIN 23   // Relay control pin for the pump
#define VALVE_RELAY_PIN 19  // Relay control pin for the solenoid valve

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

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

  if (myData.ultrasonicObjectDetected) {
    // run TOF, Water pump, solenoid valve
    runMeasureSendWaterVolume();
  }

  if (myData.tofThresholdDetected) {
    runUltrasonic();
  }

  if (finalOutputIsDisplayed) {
    runTemp();
  }

  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime > 1000) {
    sendData();
    lastSendTime = currentTime;
  }
}

// Components Initialization-------------------------------------------
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
  lcd.createChar(0, degreeChar);
  lcd.createChar(1, superscriptMinus3);
  lcd.createChar(2, upArrow);
  lcd.createChar(3, downArrow);

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

// Keypad Functions-----------------------------------------------------
// GET PRESSED KEY
void getPressedKey() {
  char customKey = customKeypad.getKey();

  if (!customKey) return;

  Serial.print("Key Pressed: ");
  Serial.println(customKey);

  switch (customKey) {
    case '#': handleHashKey(); break;
    case '*': handleAsteriskKey(); break;
    case 'A': handleAKey(); break;
    case 'B': handleBKey(); break;
    case 'C': handleCKey(); break;
    case 'D': resetCircuit(); break;
    default: handleNumberKey(customKey); break;
  }
}

// Handle # - Enter
void handleHashKey() {
  if (gyroscopeEnterPressed) {
    if (fillWaterEnterPressed) {
      if (enteringFirstInput) {
        enteringFirstInput = false;
        inputC = inputCValue.toFloat();
        Serial.print("Input C = ");
        Serial.print(inputC);
        Serial.println(" moldm-3");
        lcd.clear();
        displayInitialMessage("Output C = ", "moldm");
      } else {
        outputC = outputCValue.toFloat();
        Serial.print("Output C = ");
        Serial.print(outputC);
        Serial.println(" moldm-3");
        lcd.clear();
        displayOutputV();
      }
    } else {
      displayInitialMessage("Input C = ", "moldm");
      fillWaterEnterPressed = true;
    }
  } else {
    // Choose Mixing or Diluting
    chooseDiluteOrMix();
    gyroscopeEnterPressed = true;
  }
}

// Handle * - Clear
void handleAsteriskKey() {
  if (inputCValue.length() > 0 && outputCValue.length() == 0) {
    clearInputCValue();
  } else if (inputCValue.length() > 0 && outputCValue.length() > 0) {
    clearOutputCValue();
  }
}

// Handle A - Choose Dilute option from dilute screen
void handleAKey() {
  if (gyroscopeEnterPressed && !finalOutputIsDisplayed) {
    fillWater();
  }
}

// Handle B - Choose Mix option from options screen
void handleBKey() {
  if (gyroscopeEnterPressed && !finalOutputIsDisplayed) {
    mixToDisplay();
  }
}

// Handle C - decimal point
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

// RESET - D is pressed on the Keypad
void resetCircuit() {
  // Clear input and output values
  inputCValue = "";
  outputCValue = "";

  // Reset input flags
  enteringFirstInput = true;
  inputCHasDecimal = false;   // Reset decimal flag for the first input
  outputCHasDecimal = false;  // Reset decimal flag for the second input

  finalOutputIsDisplayed = false;  // Flag indicating if final output is displayed

  // Reinitialize the LCD display
  lcd.clear();

  // Display the initial message for Input C
  displayInitialMessage("Input C = ", "moldm");
}

// Handle Numbers
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

// Gyroscope----------------------------------------------------------
void gyroscopeToDisplay() {
  Serial.println("Gyroscope data received");
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

  Serial.print("X:");
  Serial.print(myData.angleX);
  Serial.print("\t");
  Serial.print("Y:");
  Serial.print(myData.angleY);
  Serial.print("\t");
  Serial.print("Z:");
  Serial.println(myData.angleZ);
}

// Choose options-----------------------------------------------------
void chooseDiluteOrMix() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("A - Dilute");
  lcd.setCursor(0, 1);
  lcd.print("B - Mix");
}

void mixToDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Place liquids");
  lcd.setCursor(0, 1);
  lcd.print("in the beakers");
}

//Water Part Functions-------------------------------------------------
// TOF Sensor
float tofMeasureWaterLevel() {
  VL53L0X_RangingMeasurementData_t measure;
  float total = 0;
  int validReadings = 0;

  for (int i = 0; i < 1000; i++) {
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {  // Phase failures have incorrect data
      total += measure.RangeMilliMeter;
      validReadings++;
    } else {
      Serial.println("Out of range");
    }
  }

  if (validReadings > 0) {
    return total / validReadings;
  } else {
    Serial.println("No valid readings");
    return -1;  // Return -1 to indicate an error
  }
}

// Water measuring, Water Pump & Solenoid Valve
void runMeasureSendWaterVolume() {
  float currentWaterLevel = tofMeasureWaterLevel();
  float currentVolume = (currentWaterLevel - maxWaterLevel) * 1.694915;  // Calculate the current volume

  Serial.print("Current Water Level (mm): ");
  Serial.println(currentWaterLevel);
  Serial.print("Current Water Volume (ml): ");
  Serial.println(currentVolume);

  if (currentWaterLevel >= waterBeakerHeight) {
    Serial.println("Water volume is not enough");

    // call error handling function

    digitalWrite(PUMP_RELAY_PIN, LOW);   // Turn off pump
    digitalWrite(VALVE_RELAY_PIN, LOW);  // Close valve
    while (1)
      ;  // Stop further execution
  }

  if (currentVolume < (outputV - 2)) {
    // Turn on the pump to fill water
    Serial.println("Filling water...");
    digitalWrite(PUMP_RELAY_PIN, HIGH);   // Turn on pump
    digitalWrite(VALVE_RELAY_PIN, HIGH);  // Open valve
  } else {
    Serial.println("Threshold reached, stopping pump and closing valve");
    digitalWrite(PUMP_RELAY_PIN, LOW);   // Turn off pump
    digitalWrite(VALVE_RELAY_PIN, LOW);  // Close valve
    myData.tofThresholdDetected = true;
  }

  delay(100);  // Small delay to allow sensor reading stability
}

// Display Functions---------------------------------------------------
// Display Function 0 - Fill Water
void fillWater() {
  lcd.clear();
  lcd.print("Fill all Liquids");

  unsigned long startTime = millis();  // Start time for timeout logic
  float currentWaterLevel = tofMeasureWaterLevel();
  int currentVolume = 0;

  while (true) {
    currentWaterLevel = tofMeasureWaterLevel();
    currentVolume = (waterBeakerHeight - currentWaterLevel) * 1.694915;

    if (currentWaterLevel == -1) {
      lcd.clear();
      lcd.print("Error: Out of range");
      Serial.println("Error: ToF sensor out of range");
      warning("Error: ToF sensor");
      break;
    }

    lcd.setCursor(0, 1);
    lcd.print("Water Level:");
    lcd.print(currentVolume);
    lcd.print(" mm");
    delay(500);

    // Check for timeout (e.g., 2 minutes)
    if (millis() - startTime > 120000) {  // 120000 ms = 2 minutes
      lcd.clear();
      lcd.print("Timeout: Max");
      lcd.setCursor(0, 1);
      lcd.print("water level not reached");
      Serial.println("Timeout: Max water level not reached");
      warning("Timeout: Max water level not reached");
      break;
    }

    if (currentWaterLevel >= maxWaterLevel) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Max Water Level");
      lcd.setCursor(0, 1);
      lcd.print("Reached!");
      Serial.println("Max water level reached.");
      delay(3000);
    }
  }
}

// Display Function 1 - Initial Message
void displayInitialMessage(String topRowMessage, String bottomRowMessage) {
  // Clear the screen
  lcd.clear();


  // Set the cursor and print the top row message
  lcd.setCursor(0, 0);
  lcd.print(topRowMessage);

  // Set the cursor and print the bottom row message if available
  if (bottomRowMessage.length() > 0) {
    lcd.setCursor(0, 1);
    lcd.print("(");
    lcd.print(bottomRowMessage);
    lcd.write(byte(1));
    lcd.print(")");
  }
}

// Display Function 2 - Output
void displayOutputV() {
  // Clear the screen
  lcd.clear();

  // Perform a calculation
  outputV = (inputC * 0.005) / outputC;

  // Convert outputV to a string with 6 decimal places
  dtostrf(outputV, 9, 5, outputVStr);

  // Display the final output
  lcd.setCursor(0, 0);
  lcd.print("Vo = ");
  lcd.print(outputVStr);
  lcd.print("ml");

  Serial.print("Output V = ");
  Serial.print(outputV);
  Serial.println("ml");

  runTemp();
  finalOutputIsDisplayed = true;
  myData.finalOutputIsDisplayed = finalOutputIsDisplayed;
}

// TEMPERATURE SENSOR----------------------------------------------------
void runTemp() {
  float previousTemp = 0.0;

  // Store the previous temperature
  previousTemp = temp;

  // Request temperature readings
  sensors.requestTemperatures();

  // Fetch and print the temperature in Celsius
  temp = sensors.getTempCByIndex(0);

  // Check if reading was successful
  if (temp != DEVICE_DISCONNECTED_C) {
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");

    lcd.setCursor(0, 1);
    lcd.print("Temp = ");
    lcd.print(temp);

    // Display the custom degree symbol
    lcd.write(byte(0));
    lcd.print("C");
  } else {
    Serial.println("Error: Could not read temperature data");
    lcd.setCursor(0, 1);
    lcd.print("Temp: No Device");
  }

  // Calculate temperature difference
  float tempDifference = temp - previousTemp;

  // Determine the result based on temperature difference
  if (tempDifference > 0) {
    lcd.setCursor(15, 1);
    lcd.write(byte(2));
  } else if (tempDifference < 0) {
    lcd.setCursor(15, 1);
    lcd.write(byte(3));
  }

  // Wait for a second before requesting new data
  delay(100);
}

// Ultrasonic to detect liquid levels
void runUltrasonic() {
  // Clear the trigPin by setting it LOW
  digitalWrite(trigPinUS, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by setting the trigPin HIGH for 10 microseconds
  digitalWrite(trigPinUS, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinUS, LOW);

  // Read the echoPin, which returns the duration of the pulse in microseconds
  duration = pulseIn(echoPinUS, HIGH);

  // Calculate the distance in centimeters
  distance = duration * 0.034 / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(1000);
}

// Data Communication----------------------------------------------------
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
}

void sendData() {
  esp_err_t result = esp_now_send(espBoardB_address, (uint8_t *)&myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

// Warning function
void warning(const char *message) {
  // Activate buzzer and LED

  // Display warning message on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Warning:");
  lcd.setCursor(0, 1);
  lcd.print(message);

  // Send warning message to the other ESP32 board
  struct_message warningData = myData;
  warningData.finalOutputIsDisplayed = true;  // Indicate a warning state
  esp_now_send(espBoardB_address, (uint8_t *)&warningData, sizeof(warningData));

  // Keep the warning state for 5 seconds
  delay(5000);

  // Return to previous state or reset as needed
  resetCircuit();
}