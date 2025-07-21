//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot                                          *//
//*                                                      *//
//*  Example Code for Sending Signed Integers over I2C   *//
//*  Upstairs ESP32 (Master) to Mainboard ESP32 (Slave)  *//
//*  Master Code                                         *//
//********************************************************//

// read through all of the code and the comments before asking for help
// research 'two's complement' if this is not familiar to you as it is used to represented signed (i.e. positive and negative) values
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <NewPing.h>


#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
#define TRIGGER_PIN 18
#define ECHO_PIN 5
#define MAX_DISTANCE 200

Adafruit_MPU6050 mpu;
// Constants for complementary filter
// Constants for complementary filter
float alpha = 0.98;
float dt = 0.01; // 10ms

// Variables for sensor readings
float accAngleZ;
float gyroAngleZ = 0;
float yaw = 0;
unsigned long prevTime, currTime;

// Gyroscope bias for Z-axis (calibration value)
float gyroBiasZ = 0;


NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


void setup()
{
  Serial.begin(115200);
  Serial.println("Hello, ESP32!");
  // Initialize the MPU-6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1);
  }
  Serial.println("MPU6050 initialized.");

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibrate the gyroscope
  calibrateGyroscope();

  prevTime = millis();
  Wire.begin();   // join i2c bus (address optional for the master) - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)
}

// the minimum and maximum values here are determined by the amount of bits used by the chosen variable type
// for int, this is either 16-bits or 32-bits
// due to two's complement, the minimum value is -2^(N-1), and the maximum is (2^(N-1))-1; where N is the number of bits
float angle = 0;
int distance;
int state = 0;

//state 0 -> car goes forward, checking the distance 
//state 1 -> car stops and rotates 90 degrees


void loop()
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  //Distance Loop
  distance = sonar.ping_cm();

  
   currTime = millis();
  dt = (currTime - prevTime) / 1000.0;
  prevTime = currTime;

  // Read sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Subtract gyroscope bias
  float correctedGyroZ = g.gyro.z - gyroBiasZ;

  // Calculate angle from gyroscope
  gyroAngleZ += correctedGyroZ * dt * 180 / PI;

  // Apply complementary filter for yaw
  yaw = alpha * (yaw + correctedGyroZ * dt * 180 / PI) + (1 - alpha) * gyroAngleZ;

  // Print data to Serial Plotter
  Serial.print("Raw Yaw: ");
  Serial.print(gyroAngleZ);
  Serial.print(", Filtered Yaw: ");
  Serial.println(yaw);

 // Small delay for data stability

  
  if(trunc(yaw)==0)
  {
    gyroAngleZ = 0;
    yaw = 0;
  }
  
  angle = yaw;


  if (distance > 10 && angle == 0)
  {
   state = 0;
   Serial.print("distance: ");  
   Serial.print(distance);
   Serial.print("  ");
   Serial.println(state);
   Wire.write((byte)(state & 0x000000FF));
   delay(10);
  }
  else {
    state = 1;
    Serial.print("distance: ");  
    Serial.print(distance);
    Serial.print("  ");
    Serial.println(state);  
    Wire.write((byte)(state & 0x000000FF));
    delay(10);
  }

  if (trunc(angle) > 90){
    angle = 0;
    yaw = 0;
    gyroAngleZ = 0;
  }


  
  Wire.endTransmission();   // stop transmitting
  delay(100);
}

void calibrateGyroscope() {
  const int numSamples = 100;
  float sumZ = 0;

  Serial.println("Calibrating gyroscope... Keep the sensor still.");
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumZ += g.gyro.z;
    delay(10);
  }

  gyroBiasZ = sumZ / numSamples;
  Serial.print("Gyroscope bias (Z-axis): ");
  Serial.println(gyroBiasZ, 6);
}
