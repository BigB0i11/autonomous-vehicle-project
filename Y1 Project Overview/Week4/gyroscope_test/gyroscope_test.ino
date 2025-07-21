#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <math.h>

const byte ledPin = 33;
Adafruit_MPU6050 mpu;

float gyroOffsets[3] = {0, 0, 0};
float angleX = 0, angleY = 0, angleZ = 0; // Absolute angles in degrees
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);
    Serial.println("test");
    Serial.flush();
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10); // Loop here forever if initialization fails
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(ledPin, OUTPUT);

  prevTime = millis();
  calibrateSensor();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0; // Delta time in seconds
  prevTime = currentTime;

  // Correct gyroscope readings
  float correctedGyroX = g.gyro.x - gyroOffsets[0];
  float correctedGyroY = g.gyro.y - gyroOffsets[1];
  float correctedGyroZ = g.gyro.z - gyroOffsets[2];

  // Convert gyroscope data to degrees per second
  correctedGyroX *= (180 / M_PI);
  correctedGyroY *= (180 / M_PI);
  correctedGyroZ *= (180 / M_PI);

  // Calculate accelerometer angles in degrees
  float accelAngleX = atan2(a.acceleration.y, a.acceleration.z) * (180 / M_PI);
  float accelAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * (180 / M_PI);

  // Integrate gyroscope data to track rotation
  angleX += correctedGyroX * dt;
  angleY += correctedGyroY * dt;
  angleZ += correctedGyroZ * dt;

  // Complementary filter to combine accelerometer and gyroscope data
  const float alpha = 0.96; // Weight for gyroscope (tunable)
  angleX = alpha * angleX + (1 - alpha) * accelAngleX;
  angleY = alpha * angleY + (1 - alpha) * accelAngleY;

  // Print absolute angles in degrees
  Serial.print("Absolute Angle X: ");
  Serial.print(angleX, 2);
  Serial.print("°, Y: ");
  Serial.print(angleY, 2);
  Serial.print("°, Z: ");
  Serial.print(angleZ, 2);
  Serial.println("°");

  // Example: Activate LED when Y-angle reaches 90 degrees
  if ((angleZ >= 90) && (angleZ < 91)) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  delay(10); // Keep the sampling rate consistent
}

void calibrateSensor() {
  const int sampleCount = 500;
  float gyroSum[3] = {0, 0, 0};

  for (int i = 0; i < sampleCount; i++) {
    sensors_event_t accel, g, temp;
    mpu.getEvent(&accel, &g, &temp);

    gyroSum[0] += g.gyro.x;
    gyroSum[1] += g.gyro.y;
    gyroSum[2] += g.gyro.z;

    delay(10);
  }

  gyroOffsets[0] = gyroSum[0] / sampleCount;
  gyroOffsets[1] = gyroSum[1] / sampleCount;
  gyroOffsets[2] = gyroSum[2] / sampleCount;

  Serial.println("Gyroscope calibration complete.");
}
