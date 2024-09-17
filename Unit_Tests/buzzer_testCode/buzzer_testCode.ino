const int buzzerPin = 33; // Pin connected to the buzzer

void setup() {
  pinMode(buzzerPin, OUTPUT); // Set buzzer pin as output
}

void loop() {
  shortRepeatedBeeps(); // Play short repeated beeps
  delay(0);          // Wait for 5 seconds before repeating
}

// Function to play short repeated beeps
void shortRepeatedBeeps() {
  int toneFrequency = 1000; // Frequency of the tone in Hz
  int toneDuration = 100;   // Duration of each tone in milliseconds
  int pauseDuration = 100;  // Pause between tones in milliseconds

  for (int i = 0; i < 5; i++) { // Play 5 beeps
    tone(buzzerPin, toneFrequency); // Play tone
    delay(toneDuration);           // Wait for tone duration
    noTone(buzzerPin);             // Stop the tone
    delay(pauseDuration);          // Wait between beeps
  }
}