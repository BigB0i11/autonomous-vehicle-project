#include <LiquidCrystal.h>
#include <ESP32PWM.h>
LiquidCrystal lcd(19,23,5,18,15,21);
void setup()
{
    lcd.begin(16, 2);
    lcd.clear();
    lcd.print("How to Interface");

    // go to row 1 column 0, note that this is indexed at 0
    lcd.setCursor(0,1); 
    lcd.print ("LCD with ESP32");
}
void loop(){}