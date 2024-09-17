#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {

  Serial.begin(115200);

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  Serial.println(F("VL53L0X started"));
}

void loop() {
    Serial.println("----------------------------------");
  float realHeight = tofMeasureWaterLevel();

  if (realHeight != -1) {
    //Serial.print("Apparent Height (mm): ");
    Serial.println(realHeight / 1.33); // Calculate and print the apparent height
    Serial.print("Real Height (mm): ");
    Serial.println(realHeight); // Print the corrected real height
  } else {
    Serial.println("Error: No valid readings");
  }

  delay(1000); // Delay to avoid rapid looping
}

float tofMeasureWaterLevel() {
  VL53L0X_RangingMeasurementData_t measure;
  float total = 0;
  int validReadings = 0;

  for (int i = 0; i < 30; i++) {
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {  // Phase failures have incorrect data
      total += measure.RangeMilliMeter;
      validReadings++;
    } else {
      Serial.println("Out of range");
    }
  }

  if (validReadings > 0) {
    float apparentHeight = total / validReadings;
    float realHeight = apparentHeight * 1.33; // Correct the height using the refractive index
    return realHeight;
  } else {
    Serial.println("No valid readings");
    return -1;  // Return -1 to indicate an error
  }
}
