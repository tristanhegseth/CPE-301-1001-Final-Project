#include <LiquidCrystal.h>

const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

void setup() {
  // put your setup code here, to run once:
  lcd.begin (20, 4);  //set up the LCD's number of columns

//  if (pin A0<512)
  lcd.setCursor (1,0);
  lcd.write("Water level is");
  lcd.setCursor(5,1);
  lcd.write("low!!");
  

}

void loop() {
   lcd.setCursor(0, 1);
 //  lcd.print(millis() / 1000);
  

}
