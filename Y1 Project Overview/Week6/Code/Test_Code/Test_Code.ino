// Define pin connections
#define JOYSTICK_X_PIN 14
#define JOYSTICK_Y_PIN 32
#define JOYSTICK_BTN_PIN 27
#define BUZZER_PIN 33
#define LED_PIN 25 // Optional LED

// Threshold for joystick movement detection
#define JOYSTICK_THRESHOLD 2000

void setup() {
    pinMode(JOYSTICK_BTN_PIN, INPUT_PULLUP);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
}

void loop() {
    int xValue = analogRead(JOYSTICK_X_PIN);
    int yValue = analogRead(JOYSTICK_Y_PIN);
    int buttonState = digitalRead(JOYSTICK_BTN_PIN);
    
    Serial.print("X: "); Serial.print(xValue);
    Serial.print(" | Y: "); Serial.print(yValue);
    Serial.print(" | Button: "); Serial.println(buttonState);
    
    // Check if the joystick is moved beyond threshold
    if (abs(xValue - 2048) > JOYSTICK_THRESHOLD || abs(yValue - 2048) > JOYSTICK_THRESHOLD) {
        tone(BUZZER_PIN, 1000); // Play sound at 1kHz
        digitalWrite(LED_PIN, HIGH); // Turn on LED
    } else {
        noTone(BUZZER_PIN); // Stop buzzer
        digitalWrite(LED_PIN, LOW); // Turn off LED
    }
    
    delay(100);
}

