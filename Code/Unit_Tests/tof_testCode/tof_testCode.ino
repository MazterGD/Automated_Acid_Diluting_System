#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <LiquidCrystal_I2C.h>

// Create an instance of the VL53L0X sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adjust the I2C address based on your setup

byte fillChar[8] = {
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111
};

void setup() {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, fillChar);

  Serial.println(F("Serial communication started"));

  // Initialize I2C communication
  Wire.begin();
  Serial.println(F("I2C communication started"));

  // Initialize the VL53L0X sensor
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  Serial.println(F("VL53L0X started"));
}

void loop() {
  Serial.println(F("Starting measurement"));
  float currentWaterLevel = tofMeasureWaterLevel();
  Serial.print(F("Current water level: "));
  Serial.println(currentWaterLevel);

  float beakerHeight = 250.0;
  float fillPercentage = (currentWaterLevel / beakerHeight) * 100;
  int screenPercentage = (int)(16 * fillPercentage / 100);  // Adjust fill percentage to LCD width

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Water Level: ");
  lcd.setCursor(13, 0);
  lcd.print(fillPercentage, 1);  // Print fill percentage with 1 decimal place
  lcd.print("%");

  for (int i = 0; i < screenPercentage; i++) {
    lcd.setCursor(i, 1);
    lcd.print(byte(0));
  }

  // Delay for a short while before taking the next measurement
  delay(500);
}

float tofMeasureWaterLevel() {
  VL53L0X_RangingMeasurementData_t measure;
  float total = 0;
  int validReadings = 0;

  for (int i = 0; i < 100; i++) {
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {  // Phase failures have incorrect data
      total += measure.RangeMilliMeter;
      validReadings++;
    } else {
      Serial.println(F("Out of range"));
    }
  }

  if (validReadings > 0) {
    Serial.println(F("Valid readings obtained"));
    return total / validReadings;
  } else {
    Serial.println(F("No valid readings"));
    return -1;  // Return -1 to indicate an error
  }
}
