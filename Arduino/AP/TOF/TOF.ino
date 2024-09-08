/*
  Reading distance from the laser based VL53L0X-V2
  Using the Adafruit VL53L0X library
  This example prints the distance to an object in millimeters and feet.

  Make sure you have installed the Adafruit VL53L0X library.
*/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Create an instance of the sensor
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup()
{
  // Initialize the I2C communication
  Wire.begin();

  // Start serial communication for debugging
  Serial.begin(115200);
  Serial.println("Adafruit VL53L0X test");

  // Initialize the VL53L0X sensor
  if (!lox.begin())
  {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }
  Serial.println(F("VL53L0X online!"));
}

void loop()
{
  VL53L0X_RangingMeasurementData_t measure;

  // Start ranging
  lox.rangingTest(&measure, false); // Pass in 'true' to get debug data printout!

  // Check if the sensor returns a valid distance measurement
  if (measure.RangeStatus != 4) // phase failures have inconsistent data
  {
    int distance = measure.RangeMilliMeter;
    Serial.print("Distance(mm): ");
    Serial.print(distance);

    float distanceInches = distance * 0.0393701;
    float distanceFeet = distanceInches / 12.0;

    Serial.print("\tDistance(ft): ");
    Serial.print(distanceFeet, 2);

    Serial.println();
  }
  else
  {
    Serial.println("Out of range");
  }

  delay(1000); // Delay for a second before the next measurement
}
