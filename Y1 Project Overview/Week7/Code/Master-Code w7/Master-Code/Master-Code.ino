// Include necessary libraries
#include <LiquidCrystal.h>
#include <Keypad.h>
#include <WiFi.h>

// Initialize the LCD (use safe GPIOs)
LiquidCrystal lcd(19, 23, 18, 5, 16, 15);  
// Define the Keypad size
const byte ROWS = 4;  // Four rows
const byte COLS = 3;  // Three columns

// Define the key layout
char keys[ROWS][COLS] = {
 {'1', '2', '3'},
 {'4', '5', '6'},
 {'7', '8', '9'},
 {'*', '0', '#'}
};

byte rowPins[ROWS] = {4, 21, 17, 25}; 
byte colPins[COLS] = {26, 14, 27};      

// Initialize the keypad
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

uint8_t slaveMAC[] = {0x08, 0xB6, 0x1F, 0xEF, 0xB8, 0xF0}; 
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
  Serial.begin(115200);  
  lcd.begin(16, 2); 
  lcd.clear();  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();   
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
  char key = keypad.getKey();  // Get the pressed key

  if (key) {  // Only process if a key is actually pressed
    Serial.println(key);
    valuesToSend.data1 = key;
    valuesToSend.data2 = 0; 
    lcd.setCursor(0, 1);
    lcd.print("State:");
    lcd.setCursor(1, 1);
    lcd.print(key) 
  }
  esp_err_t result = esp_now_send(slaveMAC, (uint8_t *) &valuesToSend, sizeof(valuesToSend));

    if (result == ESP_OK) {
        Serial.println("Sent with success");
    } else {
        Serial.println("Error sending the data");
    }
delay(100);
}
