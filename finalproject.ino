#include <LiquidCrystal.h>
#include <Servo.h>
#include <dht_nonblocking.h>//for temp and humidity
#define DHT_SENSOR_TYPE DHT_TYPE_11 

Servo vent; //Vent angle servo motor


 // Define Port K Register Pointers


int vent_motor_pin = 6;
int vent_control_pin = A1;    // select the input pin for the potentiometer
int vent_control_channel = vent_control_pin - 54; //only on ATmega2560
int vent_input_data = 0;
float vent_angle = 0;    // variable to store the servo position


static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );
float a = 9;      //temp conversion variable a and b
float b = 5;
float temperature = 0;
float humidity = 0;

const int sensorPin = A0;

int HistoryValue = 0;
char printBuffer[128];// to print water level to serial moniter

int  temp=0;
int Humidity=0;

const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

void setup() {

  lcd.begin (16, 2); 
  Serial.begin(9600);

  adc_init(); //Enable Analog to Digital Read
  vent.attach(vent_motor_pin); //Servo pin initializatio  

   //  pinMode (1, INPUT_PULLUP); //pushbutton
}

void loop() {


  vent_input_data = adc_read(vent_control_channel); //read from ADC channel
  vent_angle = vent_input_data / 1000.f * 180.f; //Input ranges [0, 1000] -> Servo ranges [0, 180] conversion
  vent.write(vent_angle); 

//prints temp and humidity to serial moniter
  dht_sensor.measure(&temperature, &humidity);
float   tempF = (((temperature * (a / b)) + 32)); //celcius to F conversion
  Serial.print( "T = " );
  Serial.print( tempF, 1 );
  Serial.print( " deg. F, H = " );
  Serial.print( humidity, 1 );
  Serial.println( "%" );
 // delay(3000);
 
  int value = analogRead(sensorPin);  // get adc value 
  int diffValue = HistoryValue - value;
if (value<100)
{
  lcd.setCursor (0, 0);
  lcd.clear ();
  lcd.setCursor (1,0);
  lcd.write("Water level is");
  lcd.setCursor(5,1);
  lcd.write("low!");
  digitalWrite (5, HIGH); // turn on red LED
  digitalWrite (4, LOW);// turn off blue light
}
 else{

   lcd.setCursor (0, 0);
      lcd.clear ();
      lcd.write("Temp:  ");
      lcd.print(tempF);
       lcd.write("F  ");
      lcd.setCursor (0, 1);
      lcd.write("Humidity: ");
      lcd.print(humidity);
      lcd.write("%");
      digitalWrite (5, LOW); // turn off red LED
digitalWrite (4, HIGH); // blue led on
      
 }
 //


    
// print water level to serial moniter
 
    if((abs(diffValue)>=10 ) || ((value == 0) &&(HistoryValue !=0)))
    {
      sprintf(printBuffer,"Water level is %d\n", value);
      Serial.print(printBuffer);
      HistoryValue = value;
    }
    



}// void loop

void adc_init() {
  ADCSRA |= (0x01 << ADEN); //Enable ADC
}

unsigned int adc_read(unsigned char adc_channel) {
  unsigned int high, low = 0; //Have to read high low in correct order or it breaks

  ADMUX = (0x01 << 6) | (adc_channel & 0x07); //Set ADLAR to 1, select correct channel to read

  ADCSRA |= (0x01 << ADSC); //Start conversion
  while (ADCSRA & (0x01 << ADSC)); //check for conversion completion

  low = ADCL; //read value
  high = ADCH;
  return (high << 8) | low; //return value
}
