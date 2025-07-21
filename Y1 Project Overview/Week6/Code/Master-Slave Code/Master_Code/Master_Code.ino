// Define pin connections
#include <esp_now.h>
#include <WiFi.h>
#define JOYSTICK_X_PIN 35
#define JOYSTICK_Y_PIN 32
#define JOYSTICK_BTN_PIN 27
#define BUZZER_PIN 33
#define LED_PIN 25

// Threshold for joystick movement detection
#define JOYSTICK_THRESHOLD 2000
uint8_t slaveMAC[] = {0x08, 0xB6, 0x1F, 0xEF, 0xB8, 0xF0}; 
int D = 0;
String success;
typedef struct struct_message {
    int data1;
    int data2;
    int data3;
} struct_message;
struct_message valuesToSend;
esp_now_peer_info_t peerInfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}


void setup() {
    pinMode(JOYSTICK_BTN_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();  // Ensure Wi-Fi is disconnected before using ESP-NOW

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    memset(&peerInfo, 0, sizeof(peerInfo));  // Clear structure before use
    memcpy(peerInfo.peer_addr, slaveMAC, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    int xValue = analogRead(JOYSTICK_X_PIN);
    int yValue = analogRead(JOYSTICK_Y_PIN);
    int buttonState = digitalRead(JOYSTICK_BTN_PIN);

    // //Determine direction
    if (yValue > 2048 + JOYSTICK_THRESHOLD) {
        D = 1;
    } else if (yValue < 2048 - JOYSTICK_THRESHOLD) {
        D = 2;
    } else if (xValue > 2048 + JOYSTICK_THRESHOLD) {
        D = 3;
    } else if (xValue < 2048 - JOYSTICK_THRESHOLD) {
        D = 4;
    } else if (buttonState == 0) {  
        D = 5;
        tone(BUZZER_PIN, 1000); 
        digitalWrite(LED_PIN, HIGH);
    } else {
        D = 0;
        noTone(BUZZER_PIN);  // Stop buzzer when button is not pressed
        digitalWrite(LED_PIN, LOW);  // Turn off LED
    }

    valuesToSend.data1 = D;
    valuesToSend.data2 = 0;  // Initialize extra struct values

    Serial.println("State: ");
    Serial.println(D);
    Serial.println("Button: ");
    Serial.println(buttonState);
    valuesToSend.data3 = 0;

    esp_err_t result = esp_now_send(slaveMAC, (uint8_t *) &valuesToSend, sizeof(valuesToSend));

    if (result == ESP_OK) {
        Serial.println("Sent with success");
    } else {
        Serial.println("Error sending the data");
    }

    delay(100);
}
