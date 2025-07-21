//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot                                          *//
//*                                                      *//
//*  Example Code for Sending Signed Integers over I2C   *//
//*  Upstairs ESP32 (Master) to Mainboard ESP32 (Slave)  *//
//*  Slave Code                                         *//
//********************************************************//

// read through all of the code and the comments before asking for help
// research 'two's compliment' if this is not familiar to you as it is used to represented signed (i.e. positive and negative) values

#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal

#include <ESP32Servo.h>

Servo steeringServo;

#define enA 33  // enableA command line
#define enB 25  // enableB command line

#define INa 26  // channel A direction
#define INb 27  // channel A direction
#define INc 14  // channel B direction
#define INd 12  // channel B direction

// setting PWM properties
const int freq = 2000;
const int ledChannela = 1;  // the ESP32 servo library uses the PWM channel 0 by default, hence the motor channels start from 1
const int ledChannelb = 2;
const int resolution = 8;

int steeringAngle = 91;    // variable to store the servo position
int servoPin = 13;  // the servo is attached to IO_13 on the ESP32



void setup()
{
  Wire.begin(I2C_SLAVE_ADDR);   // join i2c bus #4 - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)
  Wire.onReceive(onReceive);    // register event
  // configure the LED PWM functionalitites and attach the GPIO to be controlled - ensure that this is done before the servo channel is attached
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

  Serial.begin(9600);           // start serial for output
  Serial.println("ESP32 Running");
  steeringServo.write(steeringAngle);

}

int leftSpeed = 0;
int rightSpeed = 0;

void loop()
{
  Serial.println("Looping"); 
  delay(100);
}



// this function executes whenever data is received from master device
void onReceive(int howMany)
{

  int state;

  

  state = Wire.read();  
  Serial.print("Printing:"); 
  Serial.println(state);  
   

  if (state == 0){
    steeringServo.write(91);
    goForwards();
    motors(95,254);
    delay(10);


  }
  if (state == 1){
    stopMotors();
    delay(10);
    steeringServo.write(0);
    goAntiClockwise();
    
  }


}

// function to clear the I2C buffer
void emptyBuffer(void)
{
  Serial.println("Error: I2C Byte Size Mismatch");
  while(Wire.available())
  {
    Wire.read();
  }
}

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

void goForwards() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
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