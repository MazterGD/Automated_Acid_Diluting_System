// |———————————————————————————————————————————————————————| 
// |  made by Arduino_uno_guy 11/13/2019                   |
// |   https://create.arduino.cc/projecthub/arduino_uno_guy|
// |———————————————————————————————————————————————————————|

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Initialize the liquid crystal library
// The first parameter is the I2C address
// The second parameter is how many columns are on your screen
// The third parameter is how many rows are on your screen
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Initialize lcd screen
  lcd.init();
  // Turn on the backlight
  lcd.backlight();
}

void loop() {
  // Wait for a second
  delay(1000);
  // Tell the screen to write on the top row
  lcd.setCursor(0, 0);
  // Tell the screen to write "Hello, From" on the top row
  lcd.print("Welawa 2.34am");
  // Tell the screen to write on the bottom row
  lcd.setCursor(0, 1);
  // Tell the screen to write "Arduino_uno_guy" on the bottom row
  // You can change what's in the quotes to be what you want it to be!
  lcd.print("Mazter GD");

  

}