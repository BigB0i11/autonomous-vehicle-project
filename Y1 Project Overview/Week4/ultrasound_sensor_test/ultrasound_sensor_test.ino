#include <ESP32Servo.h>
#include <NewPing.h>
// Initialize sensor that uses digital pins 13 and 12.
#define TRIGGER_PIN  18  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     5 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200
const byte ledPin = 33;
int brightness = 128;
int delay_time = 10;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
void setup () {
Serial.begin(115200);  // We initialize serial connection so that we could print values from sensor.
analogWriteFrequency(ledPin, 500); // Set PWM frequency to 500 Hz
analogWriteResolution(ledPin, 8);         // Set resolution to 8-bit (0-255)
analogWrite(ledPin, brightness);  // Set initial brightness
}
void loop () {
    // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
    float distance = sonar.ping_cm();
  delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");

  if (distance <= 1)
  {
    for(int duty = 0; duty <= 255 ; duty ++)
    {
      analogWrite(ledPin, duty);
      delay(delay_time);
    }
  }

  if ((distance >= 1) && (distance < 10) )
  {
    int duty = map(distance, 10, 1, 0, 255); // Gradually dim as distance increases
    analogWrite(ledPin, duty);
  }

  if(distance >= 10)
  {
    int duty = 0;
    analogWrite(ledPin, duty);
  }
}



