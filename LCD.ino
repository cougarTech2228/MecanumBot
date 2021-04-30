// Include the libraries:
// LiquidCrystal_I2C.h: https://github.com/johnrickman/LiquidCrystal_I2C
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD

void lcdSetup() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  lcd.setCursor(0, 0);
  lcd.print("Front: ");
  lcd.setCursor(0, 1);
  lcd.print("Right: ");
  lcd.setCursor(0, 2);
  lcd.print("Rear: ");  
  lcd.setCursor(0, 3);
  lcd.print("Left: ");
}
