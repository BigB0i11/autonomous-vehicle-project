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
#define Min_DISTANCE_TIME 128
#define Mid_DISTANCE_TIME 255
#define Max_DISTANCE_TIME 383

// setting PWM properties
const int freq = 2000;
const int ledChannela = 11;  // the ESP32 servo library uses the PWM channel 0 by default, hence the motor channels start from 1
const int ledChannelb = 12;
const int resolution = 8;

int steeringAngle = 0;    // variable to store the servo position
int servoPin = 13;  // the servo is attached to IO_13 on the ESP32
int incomingVal1;
int State = 0;
typedef struct struct_message {
    int data1;
    int data2;
    int data3;
} struct_message;
struct_message incomingValues;
int max_angle = 0;
int min_angle = 0;

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    memcpy(&incomingValues, incomingData, sizeof(incomingValues));
    Serial.print("Bytes received: ");
    State = incomingValues.data1;  // Correctly update d
    Serial.println(State);
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
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initialising ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

}

void loop() {
  // this code rotates the steering between 0 and 180 degrees before driving both wheels forwards and backwards followed by rotating the EEEBot clockwise and anticlockwise for 3 seconds each time

  // set the speed of the motors - minimum speed of 0, maximum speed of 255 i.e. largest value for an 8-bit PWM
  int leftSpeed = 255;
  int rightSpeed = 255; 

  // Use a switch-case to control robot movement based on received direction value 'd'
  switch (State) {
   case 1:
      Serial.println(State);
      Centralise();
      goForwards();
      delay(Min_DISTANCE_TIME);
      stopMotors();
      break;
      
  case 2:
      Serial.println(State);
      Centralise();
      goForwards();
      delay(Mid_DISTANCE_TIME);
      stopMotors();
      break;
      
  case 3:
      Serial.println(State);
      Centralise();
      goForwards();
      delay(Max_DISTANCE_TIME);
      stopMotors();
      break;
      
  case 4:
      Serial.println(State);
      min_angle = 0;
      max_angle = 90;
      moveSteering();
      break;
      
  case 5:
      Serial.println(State);
      min_angle = 0;
      max_angle = 180;
      moveSteering();
      break;
  case 6:
    Serial.println(State);
     min_angle = 0;
     max_angle = 90;
     moveSteeringL();
     break;
  case 7:
   Serial.println(State);
   min_angle = 0;
   max_angle = 180;
   moveSteeringL();
   break;
  case 8:m
   Serial.println(State);
   goClockwise();
   delay(Min_DISTANCE_TIME);
   stopMotors();
   break;
  case 9:
   Serial.println(State); 
   goAntiClockwise();
   delay(Min_DISTANCE_TIME);
   stopMotors();
   break; 
  case 0:
   Serial.println(State);
   stopMotors();
  }
}

// Function to move steering to the right
void moveSteering() {
  steeringServo.write(0);

  for (steeringAngle = min_angle; steeringAngle <= max_angle; steeringAngle += 1) {   // goes from 0 degrees to 180 degrees in steps of 1 degree
    steeringServo.write(steeringAngle);                                 // tell servo to go to position in variable 'steeringAngle'
    delay(15);                                                          // waits 15ms for the servo to reach the position
  }
}
void Centralise(){
   steeringServo.write(0);
   for (steeringAngle <= 90; steeringAngle >= 0; steeringAngle -= 1) {   // goes from 90 degrees to 0 degrees in steps of 1 degree
    steeringServo.write(steeringAngle);                                 // tell servo to go to position in variable 'steeringAngle'
    delay(15);                                                          // waits 15ms for the servo to reach the position
  }
}
void moveSteeringL() {
  steeringServo.write(180);

  for (steeringAngle = min_angle; steeringAngle <= max_angle; steeringAngle += 1) {   // goes from 0 degrees to 180 degrees in steps of 1 degree
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
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
  motors(200, 200);
}

void goBackwards() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
  motors(200, 200);
}


void goClockwise() {
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
  motors(230,180);
}

void goAntiClockwise() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
  motors(180,230);
}

void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
  motors(0,0);
}