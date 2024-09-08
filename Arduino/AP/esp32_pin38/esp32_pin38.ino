#include <esp_now.h>
#include <WiFi.h>

// MAC address of the first ESP32 (esp32_1)
uint8_t esp32_1_address[] = {0x58, 0xBF, 0x25, 0x9E, 0xBA, 0x68};  // Replace with actual MAC address of esp32_1

// Structure to hold data to be sent
typedef struct struct_message {
    char message[32];
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\nLast Packet Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("Message: ");
    Serial.println(myData.message);
}

void setup() {
    Serial.begin(115200);
    
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register send callback
    esp_now_register_send_cb(OnDataSent);

    // Register receive callback
    esp_now_register_recv_cb(OnDataRecv);
    
    // Add peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, esp32_1_address, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    // Send a message every 2 seconds
    strcpy(myData.message, "Hello from ESP32_2");
    esp_err_t result = esp_now_send(esp32_1_address, (uint8_t *) &myData, sizeof(myData));

    if (result == ESP_OK) {
        Serial.println("Sent with success");
    }
    else {
        Serial.println("Error sending the data");
    }
    
    delay(2000);
}
