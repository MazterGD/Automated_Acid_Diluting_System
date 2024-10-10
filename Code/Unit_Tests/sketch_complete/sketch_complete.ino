//code with lcd display, datapad, temperature sensor, tof, gyroscope

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050.h>



// LCD DISPLAY

// Initialize the LCD with I2C address 0x27 and screen size of 16x2
LiquidCrystal_I2C lcd(0x27, 16, 2);



// DATAPAD

// Initialize the datapad
const byte ROWS = 4;
const byte COLS = 4;

char hexaKeys[ROWS][COLS] = {
  { 'D', '#', '0', '*' },
  { 'C', '9', '8', '7' },
  { 'B', '6', '5', '4' },
  { 'A', '3', '2', '1' }
};

// Define the row and column pins
byte rowPins[ROWS] = { 12, 13, 14, 27 };
byte colPins[COLS] = { 26, 25, 33, 32 };

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

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



//TEMPERATURE SENSOR

// Data wire is connected to GPIO 18
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);



// //TOF
// // Create sensor instances
// Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// //GYROSCOPE
// MPU6050 mpu;

// // Define the built-in LED pin
// const int BUILTIN_LED = 2;



// //ULTRASONIC SENSOR
// #define echoPin 5
// #define trigPin 4
// long ultrasonicDuration, ultrasonicDistance;
// const float ultrasonicThreshold = 50.0;  // Distance threshold in cm for object detection



void resetCircuit();



void setup() {
  // SERIAL MONITOR

  // Initialize the Serial Monitor
  Serial.begin(115200);



  //LCD DISPLAY

  // Initialize the LCD screen
  lcd.init();
  // Turn on the backlight
  lcd.backlight();
  // Create a custom characters (degree symbol, superscript -3)
  lcd.createChar(0, degreeChar);
  lcd.createChar(1, superscriptMinus3);

  // Display initial message for Input C
  displayInitialMessage("Input C = ", "moldm");



  //TEMPERTURE SENSOR

  Serial.println("Dallas Temperature IC Control Library Demo");

  // Start up the library
  sensors.begin();




  // //TOF
  // // Initialize Serial Monitor
  // while (!Serial) { delay(10); }

  // // Initialize I2C communication
  // Wire.begin();

  // // Initialize the VL53L0X sensor
  // if (!lox.begin()) {
  //   Serial.println(F("Failed to boot VL53L0X"));
  //   while (1)
  //     ;
  // }
  // Serial.println(F("VL53L0X Ready"));



  // //GYROSCOPE
  // // Initialize the MPU6050 sensor
  // mpu.initialize();
  // if (!mpu.testConnection()) {
  //   Serial.println(F("Failed to connect to MPU6050"));
  //   while (1)
  //     ;
  // }
  // Serial.println(F("MPU6050 Ready"));

  // // Initialize the built-in LED pin
  // pinMode(BUILTIN_LED, OUTPUT);
  // digitalWrite(BUILTIN_LED, LOW);  // Ensure LED is off initially



  // //ULTRASONIC SENSOR
  // pinMode(trigPin, OUTPUT);
  // pinMode(echoPin, INPUT);

}



void loop() {
  getPressedKey();

  if (finalOutputIsDisplayed == true) {
    // runGyroscope();
    //runUltrasonic();
    // runTOF();
    // runWaterFlow();
    //runWaterPump();
    //runSolenoidValve();
    //runMotors();
    //runSyringeMotors();
    runTemp();
    //Base & cleaning
  }
}

//GET PRESSED KEY
void getPressedKey() {
  char customKey = customKeypad.getKey();

  if (customKey) {
    if (customKey == '#') {  // Use '#' as the delimiter to switch inputs
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

        // Both inputs are now complete
        lcd.clear();
        displayOutputV();
      }
    } else if (customKey == '*') {
      // Clear the current input
      if (inputCValue.length() > 0 && outputCValue.length() == 0) {
        inputCValue = "";
        lcd.setCursor(10, 0);
        lcd.print("          ");
        lcd.setCursor(10, 0);
      } else if (inputCValue.length() > 0 && outputCValue.length() > 0) {
        outputCValue = "";
        lcd.setCursor(11, 0);
        lcd.print("          ");
        lcd.setCursor(11, 0);
      }
    } else if (customKey == 'A') {
      if (enteringFirstInput) {
        if (!inputCHasDecimal) {  // Check if decimal point is already present
          inputCValue += ".";
          inputCHasDecimal = true;
          lcd.setCursor(10 + inputCValue.length() - 1, 0);  // Adjust cursor for new character
          lcd.print(".");
        }
      } else {
        if (!outputCHasDecimal) {  // Check if decimal point is already present
          outputCValue += ".";
          outputCHasDecimal = true;
          lcd.setCursor(11 + outputCValue.length() - 1, 0);  // Adjust cursor for new character
          lcd.print(".");
        }
      }
    } else if (customKey == 'D') {
      // Reset the circuit
      resetCircuit();
    } else {
      if (enteringFirstInput) {
        inputCValue += customKey;
        lcd.setCursor(10, 0);
        lcd.print(inputCValue);
      } else {
        outputCValue += customKey;
        lcd.setCursor(11, 0);
        lcd.print(outputCValue);
      }
    }
  }
}

// RESET
void resetCircuit() {
  // Clear input strings
  inputCValue = "";
  outputCValue = "";
  enteringFirstInput = true;

  inputCHasDecimal = false;   // Reset decimal flag for the first input
  outputCHasDecimal = false;  // Reset decimal flag for the second input

  finalOutputIsDisplayed = false;

  // Reinitialize the display
  lcd.clear();

  // Display initial message for Input C
  displayInitialMessage("Input C = ", "moldm");
}


// DISPLAY FUNCTIONS

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

  finalOutputIsDisplayed = true;
}

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

//TEMPERATURE SENSOR
void runTemp() {
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

    // Wait for a second before requesting new data
    delay(100);
}

// //GYROSCOPE SENSOR
// void runGyroscope(){
//   // Read accelerometer values from MPU6050
//   int16_t ax, ay, az;
//   mpu.getAcceleration(&ax, &ay, &az);
//   lcd.setCursor(1, 0);
//   lcd.print(ax);
//   lcd.print(", ");
//   lcd.print(ay);
//   lcd.print(", ");
//   lcd.print(az);

//   // Check if the sensor is not flat
//   // Assuming flat means ax and ay are near zero and az is positive
//   if (abs(ax) > 1000 || abs(ay) > 1000 || az < 0) {
//     // Blink the built-in LED
//     digitalWrite(BUILTIN_LED, HIGH);
//     delay(500);  // LED on for 500ms
//     digitalWrite(BUILTIN_LED, LOW);
//     delay(500);  // LED off for 500ms
//     Serial.println("Not flat");
//   } else {
//     digitalWrite(BUILTIN_LED, LOW);  // Ensure LED is off
//     Serial.println("Flat");
//   }

//   delay(100);  // Read every second
// }

// //TOF SENSOR
// void runTOF(){
//   //TOF
//   // Read tof distance from VL53L0X
//   VL53L0X_RangingMeasurementData_t measure;
//   lox.rangingTest(&measure, false);

//   if (measure.RangeStatus != 4) {  // if not out of range
//     Serial.print("TOF Distance (mm): ");
//     Serial.println(measure.RangeMilliMeter);
//   } else {
//     Serial.println("Out of range");
//   }
// }

// //ULTRASONIC SENSOR
// void runUltrasonic(){
//   digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);
  
//   ultrasonicDuration = pulseIn(echoPin, HIGH);
//   ultrasonicDistance = ultrasonicDuration / 58.2;
//   String disp = String(ultrasonicDistance);

//   // Print the ultrasonicDistance to the Serial Monitor
//   Serial.print("Ultrasonic Distance: ");
//   Serial.print(disp);
//   Serial.println(" cm");

//   // Check if the ultrasonicDistance is less than the threshold
//   if (ultrasonicDistance < ultrasonicThreshold) {
//     Serial.println("Object detected within threshold");
//   } else {
//     Serial.println("No object detected within threshold");
//   }
//   delay(1000);
// }