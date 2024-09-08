#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
}

void loop() {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert accelerometer values to G's (divide by 16384, as the range is ±2g)
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  // Calculate the tilt angles
  int angleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  int angleY = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  int angleZ = atan2(sqrt(accelX * accelX + accelY * accelY), accelZ) * 180.0 / PI;

  Serial.print("Tilt X: ");
  Serial.print(angleX);
  Serial.print(" degrees\t");

  Serial.print("Tilt Y: ");
  Serial.print(angleY);
  Serial.print(" degrees\t");

  Serial.print("Tilt Z: ");
  Serial.print(angleZ);
  Serial.println(" degrees");

  delay(1000); // Update every 500 milliseconds
}