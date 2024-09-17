#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

const char* ssid = "ESP32_AP";
const char* password = "12345678";

WiFiServer server(80);

const int ledPin = 2;  // Built-in LED pin

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Start the AP
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");

  server.begin();
  Serial.println("Server Started");
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();

    Serial.println(request);

    if (request.indexOf("ON") != -1) {
      digitalWrite(ledPin, HIGH);
    } else if (request.indexOf("OFF") != -1) {
      digitalWrite(ledPin, LOW);
    }

    // Send response to client
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("");
    client.println("<!DOCTYPE HTML>");
    client.println("<html><body><h1>ESP32 LED Control</h1></body></html>");
    client.stop();
  }
}
