// Display, Keypad, Temperature sensor, TOF sensor

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ESP32-----------------------------------------------
// MAC address of the second ESP32 (esp32_2)
uint8_t esp32_2_address[] = { 0xA0, 0xA3, 0xB3, 0x2A, 0xDE, 0x3C };  // Replace with actual MAC address of esp32_2

// Structure to hold data to be received
typedef struct struct_message {
  int angleX, angleY, angleZ;  // Gyroscope data
  // bool gyroscopeEnterPressed;  // Enter is pressed when Gyroscope is displayed
  // bool buzzerCalled;
  // bool tofThresholdDetected;
  // bool ultrasonicObjectDetected;
} struct_message;

struct_message myData;

// LCD DISPLAY---------------------------------------------
// Initialize the LCD with I2C address 0x27 and screen size of 16x2
LiquidCrystal_I2C lcd(0x27, 16, 2);

// // KEYPAD-------------------------------------------------
// // Initialize the keypad
// const byte ROWS = 4;
// const byte COLS = 4;

// char hexaKeys[ROWS][COLS] = {
//   { 'D', '#', '0', '*' },
//   { 'C', '9', '8', '7' },
//   { 'B', '6', '5', '4' },
//   { 'A', '3', '2', '1' }
// };

// byte rowPins[ROWS] = { 12, 13, 14, 27 };
// byte colPins[COLS] = { 26, 25, 33, 32 };

// Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// // Create a custom character array for the degree symbol
// byte degreeChar[8] = {
//   0b00111,
//   0b00101,
//   0b00111,
//   0b00000,
//   0b00000,
//   0b00000,
//   0b00000,
//   0b00000
// };

// // Create a custom character array for superscript minus 3 symbol
// byte superscriptMinus3[8] = {
//   0b00111,
//   0b00001,
//   0b11011,
//   0b00001,
//   0b00111,
//   0b00000,
//   0b00000,
//   0b00000
// };

// // Create a custom character array for the up arrow symbol
// byte upArrow[8] = {
//   0b00100,
//   0b01110,
//   0b11111,
//   0b11111,
//   0b01110,
//   0b01110,
//   0b01110,
//   0b01110
// };
// // Create a custom character array for the down arrow symbol
// byte downArrow[8] = {
//   0b01110,
//   0b01110,
//   0b01110,
//   0b01110,
//   0b11111,
//   0b11111,
//   0b01110,
//   0b00100
// };

// //TEMPERATURE SENSOR-------------------------------------------
// #define ONE_WIRE_BUS 4

// // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
// OneWire oneWire(ONE_WIRE_BUS);

// // Pass our oneWire reference to Dallas Temperature.
// DallasTemperature sensors(&oneWire);

// // Input Output Variabes----------------------------------------
// String inputCValue = "";
// String outputCValue = "";
// bool enteringFirstInput = true;
// bool inputCHasDecimal = false;
// bool outputCHasDecimal = false;
// bool finalOutputIsDisplayed = false;
// float inputC = 0.0;
// float outputC = 0.0;
// double outputV = 0.0;
// float temp = 0.0;
// char outputVStr[10];  // Buffer to hold the formatted outputV

// // Gyroscope Variables---------------------------------------
// bool gyroscopeEnterPressed = false;

// // ULTRASONIC Variables-------------------------------------
// const int trigPinUS1 = 25;
// const int echoPinUS1 = 26;

// // Variables to store the duration of the pulse and the distance
// long duration1;
// int distance;

// Functions-------------------------------------------------
// Gyroscope
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

  Serial.print("X:");
  Serial.print(myData.angleX);
  Serial.print("\t");
  Serial.print("Y:");
  Serial.print(myData.angleY);
  Serial.print("\t");
  Serial.print("Z:");
  Serial.println(myData.angleZ);
}

// // RESET
// void resetCircuit() {
//   // Clear input strings
//   inputCValue = "";
//   outputCValue = "";
//   enteringFirstInput = true;

//   inputCHasDecimal = false;   // Reset decimal flag for the first input
//   outputCHasDecimal = false;  // Reset decimal flag for the second input

//   finalOutputIsDisplayed = false;

//   // Reinitialize the display
//   lcd.clear();

//   // Display initial message for Input C
//   displayInitialMessage("Input C = ", "moldm");
// }

// // Display Function 1 - Initial Message
// void displayInitialMessage(String topRowMessage, String bottomRowMessage) {
//   // Clear the screen
//   lcd.clear();

//   // Set the cursor and print the top row message
//   lcd.setCursor(0, 0);
//   lcd.print(topRowMessage);

//   // Set the cursor and print the bottom row message if available
//   if (bottomRowMessage.length() > 0) {
//     lcd.setCursor(0, 1);
//     lcd.print("(");
//     lcd.print(bottomRowMessage);
//     lcd.write(byte(1));
//     lcd.print(")");
//   }
// }

// // Display Function 2 - Output
// void displayOutputV() {
//   // Clear the screen
//   lcd.clear();

//   // Perform a calculation
//   outputV = (inputC * 0.005) / outputC;

//   // Convert outputV to a string with 6 decimal places
//   dtostrf(outputV, 9, 5, outputVStr);

//   // Display the final output
//   lcd.setCursor(0, 0);
//   lcd.print("Vo = ");
//   lcd.print(outputVStr);
//   lcd.print("ml");

//   Serial.print("Output V = ");
//   Serial.print(outputV);
//   Serial.println("ml");

//   finalOutputIsDisplayed = true;
// }

// //GET PRESSED KEY
// void getPressedKey() {
//   char customKey = customKeypad.getKey();

//   if (customKey) {
//     if (customKey == '#') {  // Use '#' to proceed to next step & as the delimiter to switch inputs
//       if (gyroscopeEnterPressed == true) {
//         if (enteringFirstInput) {
//           enteringFirstInput = false;
//           inputC = inputCValue.toFloat();
//           Serial.print("Input C = ");
//           Serial.print(inputC);
//           Serial.println(" moldm-3");

//           lcd.clear();
//           displayInitialMessage("Output C = ", "moldm");
//         } else {
//           outputC = outputCValue.toFloat();
//           Serial.print("Output C = ");
//           Serial.print(outputC);
//           Serial.println(" moldm-3");

//           // Both inputs are now complete
//           lcd.clear();
//           displayOutputV();
//         }
//       } else {
//         // Display initial message for Input C
//         displayInitialMessage("Input C = ", "moldm");
//         gyroscopeEnterPressed = true;
//         myData.gyroscopeEnterPressed = gyroscopeEnterPressed;
//       }
//     } else if (customKey == '*') {
//       // Clear the current input
//       if (inputCValue.length() > 0 && outputCValue.length() == 0) {
//         inputCValue = "";
//         lcd.setCursor(10, 0);
//         lcd.print("          ");
//         lcd.setCursor(10, 0);
//       } else if (inputCValue.length() > 0 && outputCValue.length() > 0) {
//         outputCValue = "";
//         lcd.setCursor(11, 0);
//         lcd.print("          ");
//         lcd.setCursor(11, 0);
//       }
//     } else if (customKey == 'A') {
//       if (enteringFirstInput) {
//         if (!inputCHasDecimal) {  // Check if decimal point is already present
//           inputCValue += ".";
//           inputCHasDecimal = true;
//           lcd.setCursor(10 + inputCValue.length() - 1, 0);  // Adjust cursor for new character
//           lcd.print(".");
//         }
//       } else {
//         if (!outputCHasDecimal) {  // Check if decimal point is already present
//           outputCValue += ".";
//           outputCHasDecimal = true;
//           lcd.setCursor(11 + outputCValue.length() - 1, 0);  // Adjust cursor for new character
//           lcd.print(".");
//         }
//       }
//     } else if (customKey == 'D') {
//       // Reset the circuit
//       resetCircuit();
//     } else {
//       if (enteringFirstInput) {
//         inputCValue += customKey;
//         lcd.setCursor(10, 0);
//         lcd.print(inputCValue);
//       } else {
//         outputCValue += customKey;
//         lcd.setCursor(11, 0);
//         lcd.print(outputCValue);
//       }
//     }
//   }
// }

// //TEMPERATURE SENSOR
// void runTemp() {
//   // Request temperature readings
//   sensors.requestTemperatures();

//   // Fetch and print the temperature in Celsius
//   temp = sensors.getTempCByIndex(0);

//   // Check if reading was successful
//   if (temp != DEVICE_DISCONNECTED_C) {
//     Serial.print("Temperature: ");
//     Serial.print(temp);
//     Serial.println(" °C");

//     lcd.setCursor(0, 1);
//     lcd.print("Temp = ");
//     lcd.print(temp);

//     // Display the custom degree symbol
//     lcd.write(byte(0));
//     lcd.print("C");
//   } else {
//     Serial.println("Error: Could not read temperature data");
//     lcd.setCursor(0, 1);
//     lcd.print("Temp: No Device");
//   }

//   // Wait for a second before requesting new data
//   delay(100);
// }

// // Final Output Beaker detecting Ultrasonic
// void runUltrasonic1() {
//   // Clear the trigPin by setting it LOW
//   digitalWrite(trigPinUS1, LOW);
//   delayMicroseconds(2);

//   // Trigger the sensor by setting the trigPin HIGH for 10 microseconds
//   digitalWrite(trigPinUS1, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPinUS1, LOW);

//   // Read the echoPin, which returns the duration of the pulse in microseconds
//   duration1 = pulseIn(echoPinUS1, HIGH);

//   // Calculate the distance in centimeters
//   distance = duration1 * 0.034 / 2;

//   // Print the distance to the Serial Monitor
//   Serial.print("Distance: ");
//   Serial.print(distance);
//   Serial.println(" cm");

//   // Update distance data in myData
//   myData.distance = distance;
//   if (distance < 10) {
//     myData.ultrasonicObjectDetected = true;
//   }

//   delay(1000);
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
  // Serial.print("Distance: ");
  // Serial.print(myData.distance);
  // Serial.println(" cm");
  // Serial.print("Water Level: ");
  // Serial.print(myData.waterLevel);
  // Serial.println(" cm");
  gyroscopeToDisplay();
}

void sendData() {
  // Send data to second ESP32
  esp_err_t result = esp_now_send(esp32_2_address, (uint8_t *)&myData, sizeof(myData));
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
  memcpy(peerInfo.peer_addr, esp32_2_address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // LCD Display----------------------------------
  // Initialize the LCD screen
  lcd.init();
  // Turn on the backlight
  lcd.backlight();
  // Create a custom characters (degree symbol, superscript -3)
  // lcd.createChar(0, degreeChar);
  // lcd.createChar(1, superscriptMinus3);
  // lcd.createChar(2, upArrow);
  // lcd.createChar(3, downArrow);

  // // Ultrasonic---------------------------------------
  // // Set the ultrasonic sensor pins as output and input
  // pinMode(trigPinUS1, OUTPUT);
  // pinMode(echoPinUS1, INPUT);

  
}

void loop() {
  // getPressedKey();

  // if (gyroscopeEnterPressed) {
  //   // Display initial message
  //   displayInitialMessage("Input C = ", "moldm");
  // }

  // if (myData.tofThresholdDetected) {
  //   runUltrasonic1();
  // }

  // if (finalOutputIsDisplayed == true) {
  //   runTemp();
  //   myData.buzzerCalled = true;
  // }

  // Optionally control sendData() frequency
  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastSendTime > 1000) {  // Send data every 1 second
    sendData();
    lastSendTime = currentTime;
  }
}