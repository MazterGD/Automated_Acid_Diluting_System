const float waterPumpPin = 23;
const float solenoidValvePin = 19;
float desiredVolume = 0; // Desired volume in ml
float pumpRate = 13; // Pump rate in ml/s

void setup() {
  Serial.begin(115200);
  pinMode(waterPumpPin, OUTPUT);
  pinMode(solenoidValvePin, OUTPUT);
  digitalWrite(waterPumpPin, HIGH);
  digitalWrite(solenoidValvePin, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    desiredVolume = Serial.parseInt(); // Read the desired volume from serial input

    if (desiredVolume > 0) {
      int pumpTime = desiredVolume / pumpRate; // Calculate the time in seconds to pump the desired volume

      // Activate water pump and solenoid valve
      digitalWrite(waterPumpPin, LOW);
      digitalWrite(solenoidValvePin, HIGH);

      // Wait for the calculated time
      delay(pumpTime * 1000); // Convert seconds to milliseconds

      // Deactivate water pump and close solenoid valve
      digitalWrite(waterPumpPin, HIGH);
      digitalWrite(solenoidValvePin, LOW);
    }
  }
}
