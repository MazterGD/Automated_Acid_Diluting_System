#include <WiFi.h>

const char* ssid = "ESP32_AP";
const char* password = "12345678";

const int trigPin = 5;  // Trig pin of ultrasonic sensor
const int echoPin = 18; // Echo pin of ultrasonic sensor

WiFiClient client;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Connect to the Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  // Read distance from the ultrasonic sensor
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance >= 0 && distance <= 10) {
    sendRequest("ON");
  } else {
    sendRequest("OFF");
  }

  delay(1000);
}

void sendRequest(const char* state) {
  if (client.connect("192.168.4.1", 80)) {  // AP IP address
    client.print(String("GET /") + state + " HTTP/1.1\r\n" +
                 "Host: 192.168.4.1\r\n" +
                 "Connection: close\r\n\r\n");
    delay(10);
    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
    client.stop();
  }
}