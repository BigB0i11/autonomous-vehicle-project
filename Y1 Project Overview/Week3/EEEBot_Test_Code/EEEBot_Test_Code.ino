//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot                                          *//
//*                                                      *//
//*  Motor & Servo Basic Test Code                       *//
//********************************************************//

// ASSUMPTION: Channel A is LEFT, Channel B is RIGHT

// use this code to correctly assign the four pins to move the car forwards and backwards
// you first need to change the pin numbers for the four motor input 'IN' pins and two enable 'en' pins below and then 
// decide which go HIGH and LOW in each of the movements, stopMotors has been done for you
// ** marks where you need to insert the pin number or state

// feel free to modify this code to test existing or new functions

#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

Servo steeringServo;

#define enA 33  // enableA command line
#define enB 25  // enableB command line

#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction

// setting PWM properties
const int freq = 2000;
const int ledChannela = 11;  // the ESP32 servo library uses the PWM channel 0 by default, hence the motor channels start from 1
const int ledChannelb = 12;
const int resolution = 8;

int steeringAngle = 0;    // variable to store the servo position
int servoPin = 13;  // the servo is attached to IO_13 on the ESP32
int d = 0;

// Callback function to handle received data via ESP-NOW
void onReceive(const esp_now_recv_info* recvInfo, const uint8_t* data, int len) {
    // Assuming the data received is an integer
    int d = 0;
    memcpy(&d, data, sizeof(d));  // Assign received data to d
}

void setup() {
  // configure the LED PWM functionalities and attach the GPIO to be controlled
  ledcAttachChannel(enA, freq, resolution, ledChannela);
  ledcAttachChannel(enB, freq, resolution, ledChannelb);

  // allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  steeringServo.setPeriodHertz(50);    // standard 50Hz servo
  steeringServo.attach(servoPin, 500, 2400);   // attaches the servo to the pin using the default min/max pulse widths of 1000us and 2000us

  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);

  // initialise serial communication
  Serial.begin(115200);
  Serial.println("ESP32 Running");

  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onReceive); 
}

void loop() {
  // this code rotates the steering between 0 and 180 degrees before driving both wheels forwards and backwards followed by rotating the EEEBot clockwise and anticlockwise for 3 seconds each time

  // set the speed of the motors - minimum speed of 0, maximum speed of 255 i.e. largest value for an 8-bit PWM
  int leftSpeed = 0;
  int rightSpeed = 255;

  // Use a switch-case to control robot movement based on received direction value 'd'
  goForwards();
  motors(leftSpeed, rightSpeed);
  delay(1000);
  stopMotors();
}

// Function to move steering to the right
void moveSteering() {
  steeringServo.write(0);

  for (steeringAngle = 0; steeringAngle <= 146; steeringAngle += 1) {   // goes from 0 degrees to 180 degrees in steps of 1 degree
    steeringServo.write(steeringAngle);                                 // tell servo to go to position in variable 'steeringAngle'
    delay(15);                                                          // waits 15ms for the servo to reach the position
  }
}

// Motor control functions
void motors(int leftSpeed, int rightSpeed) {
  // set individual motor speed
  // the direction is set separately

  // constrain the values to within the allowable range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  ledcWrite(enA, leftSpeed);
  ledcWrite(enB, rightSpeed);
  delay(25);
}

// Motor movement functions
void goForwards() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

void goBackwards() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void goClockwise() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void goAntiClockwise() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
}

void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}
