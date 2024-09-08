#define TRIG_PIN 25
#define ECHO_PIN 26
#define TRIG_PIN2 12
#define ECHO_PIN2 13

void setup() {
  Serial.begin(115200); // Initialize Serial Monitor at 115200 baud rate
  pinMode(TRIG_PIN, OUTPUT); // Set the TRIG_PIN as an output
  pinMode(ECHO_PIN, INPUT); // Set the ECHO_PIN as an input
  pinMode(TRIG_PIN2, OUTPUT); // Set the TRIG_PIN2 as an output
  pinMode(ECHO_PIN2, INPUT); // Set the ECHO_PIN2 as an input
}

void US1() {
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
  Serial.print("Distance 1: ");
  Serial.print(distanceCm);
  Serial.println(" cm");
}

void US2() {
  // Clear the TRIG_PIN2 by setting it LOW
  digitalWrite(TRIG_PIN2, LOW);
  delayMicroseconds(2);

  // Set the TRIG_PIN2 HIGH for 10 microseconds
  digitalWrite(TRIG_PIN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN2, LOW);

  // Read the time the ECHO_PIN2 is HIGH
  long duration2 = pulseIn(ECHO_PIN2, HIGH);

  // Calculate the distance in centimeters
  float distanceCm2 = duration2 * 0.034 / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance 2: ");
  Serial.print(distanceCm2);
  Serial.println(" cm");
}

void loop() {
  US1();
  delay(500); // Small delay to avoid collision
  US2();
  delay(500); // Small delay to avoid collision
}
