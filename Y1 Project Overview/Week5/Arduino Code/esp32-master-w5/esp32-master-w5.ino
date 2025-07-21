
// Master ESP32 Code (Line Follower with Photodiodes)
#include <Wire.h>  // I2C library for communication with slave
#include <ESP32Servo.h>
#define SLAVE_ADDR 0x08  // I2C address of slave ESP32
#define LEFT_SENSOR 35  // Photodiode Left
#define RIGHT_SENSOR 33  // Photodiode Right
#define MID_LEFT_SENSOR 26  // Photodiode Mid-Left
#define MID_RIGHT_SENSOR 32 // Photodiode Mid-Right

void setup() {
    Wire.begin();   // Start I2C communication as master
    pinMode(LEFT_SENSOR, INPUT);
    pinMode(RIGHT_SENSOR, INPUT);
    pinMode(MID_LEFT_SENSOR, INPUT);
    pinMode(MID_RIGHT_SENSOR, INPUT);
}

void loop() {
    int leftValue = analogRead(LEFT_SENSOR);  // Read left sensor
    int rightValue = analogRead(RIGHT_SENSOR); // Read right sensor
    int midLeftValue = analogRead(MID_LEFT_SENSOR); // Read mid-left sensor
    int midRightValue = analogRead(MID_RIGHT_SENSOR); // Read mid-right sensor
    
    int State;
    if (midLeftValue > 1000 && midRightValue > 1000) {
        State = 1; // Move Forward
    } else if (leftValue > 1000) {
        State = 2; // Turn Left
    } else if (rightValue > 1000) {
        State = 3; // Turn Right
    } else {
        State = 4; // Stop
    }
    
    sendCommand(command);
    delay(100);
}

void sendCommand(char cmd) {
    Wire.beginTransmission(SLAVE_ADDR); // Start I2C transmission
    Wire.write(cmd);  // Send command character to slave
    Wire.endTransmission(); // End transmission
}
