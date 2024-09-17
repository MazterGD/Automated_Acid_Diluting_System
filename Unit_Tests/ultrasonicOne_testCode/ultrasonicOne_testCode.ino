#define TRIG_PIN 12
#define ECHO_PIN 13

void setup() {
  Serial.begin(115200); // Initialize Serial Monitor at 115200 baud rate
  pinMode(TRIG_PIN, OUTPUT); // Set the TRIG_PIN as an output
  pinMode(ECHO_PIN, INPUT); // Set the ECHO_PIN as an input
}

void loop() {
  // Clear the TRIG_PIN by setting it LOW
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Set the TRIG_PIN HIGH for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the time the ECHO_PIN is HIGH
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance in centimeters
  float distanceCm = duration * 0.034 / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  // Wait for a short period before taking another measurement
  delay(500);
}
