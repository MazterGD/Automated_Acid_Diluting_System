#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Create a VL53L0X object
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Refractive index of water
const float refractiveIndex = 1.333;

void setup() {
  Serial.begin(115200);
  Serial.println("VL53L0X Test");

  // Initialize sensor
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
  Serial.println(F("VL53L0X Ready"));
}

void loop() {
  const int numSamples = 100;
  float totalDistance = 0;
  int validSamples = 0;

  for (int i = 0; i < numSamples; i++) {
    VL53L0X_RangingMeasurementData_t measure;

    // Perform the measurement
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      float distance = measure.RangeMilliMeter / 10.0;  // Convert to cm
      float correctedDistance = distance * refractiveIndex;  // Apply refraction correction
      totalDistance += correctedDistance;
      validSamples++;
    }

    delay(10);  // Delay to achieve approximately 100 samples per second
  }

  if (validSamples > 0) {
    float averageDistance = totalDistance / validSamples;
    Serial.print("Average corrected distance: ");
    Serial.print(averageDistance);
    Serial.println(" cm");
  } else {
    Serial.println("No valid samples");
  }

  delay(1000 - (numSamples * 10));  // Adjust delay to complete one second cycle
}
