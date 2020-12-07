/*
* CPE 301 Final Project Team 5 - December 6, 2020
* Taylor Miller, Tristan Hegseth
* Swamp Cooler Code using Arduino Mega 2560 
*/


//LED AND MOTOR PINS
static const int    red_led = 25;
static const int yellow_led = 27;
static const int  green_led = 24;
static const int   blue_led = 26;

static const int  motor_pin = 5;


//STOP BUTTON GLOBALS
static const int stop_button_pin = A2;
int stop_button_channel = stop_button_pin - 54; //only on ATmega2560 


//LCD GLOBALS
#include <LiquidCrystal.h>

static const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


//VENT GLOBALS
#include <Servo.h>

Servo vent; //Vent angle servo motor
static const int vent_motor_pin = 6;
static const int vent_control_pin = A1;    // select the input pin for the potentiometer
int vent_control_channel = vent_control_pin - 54; //only on ATmega2560
int vent_input_data = 0;
float vent_angle = 0;    // variable to store the servo position


//TEMPERATURE AND HUMIDITY GLOBALS
#include <dht_nonblocking.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11 

static const int dht_sensor_pin = 2;
DHT_nonblocking dht_sensor(dht_sensor_pin, DHT_SENSOR_TYPE);
float temperature = 0;
float humidity = 0;


//WATER LEVEL GLOBALS
static const int water_sensor_pin = A0;
int water_sensor_channel = water_sensor_pin - 54; //only on ATmega2560
int water_level = 0;
char water_print_buffer[128];


//CLOCK GLOBALS
#include <DS3231.h>

DS3231 clock;
RTCDateTime dt;


//STATE GLOBAL
int state = 0; 
//IDLE - 0
//RUNNING - 1
//DIABLED - 2
//ERROR - 3


//ARDUINO MAIN FUNCTIONS
void setup() {
  
  pinMode(green_led, OUTPUT);
  pinMode(yellow_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(blue_led, OUTPUT);
  pinMode(motor_pin, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  
  lcd.begin (16, 2); //(Columns, Rows)

  adc_init(); //Enable Analog to Digital Read
 
  vent.attach(vent_motor_pin); //Servo pin initializatio  

  Serial.begin(9600);
  
  clock.begin();
  clock.setDateTime(__DATE__, __TIME__); //Set start time of clock
}

void loop() {
  //Move Vent
  poll_vent();
 
  //Check for stop button pushes
  int c = adc_read(stop_button_channel); 
  if (c > 500) {
    state = 2; //DISABLED
  }
  
  if (state != 2) { //state != DISABLED
    //Get Temperature(F) and Humidity
    dht_sensor.measure(&temperature, &humidity);
    //Get Water Level
    water_level = adc_read(water_sensor_channel);
  }
  
  digitalWrite(green_led, LOW);
  digitalWrite(yellow_led, LOW);
  digitalWrite(red_led, LOW);
  digitalWrite(blue_led, LOW);
 
 switch (state) {
   case 0: //IDLE
     print_temp_humidity_lcd();
     print_water_level_serial();
     digitalWrite(green_led, HIGH);
     digitalWrite(motor_pin, LOW);
     
     if ((temperature * 1.8 + 32) > 75){
       state= 1;
       print_timestamp_serial();
       Serial.println("Motor on");
     }
     if (water_level < 100){
       state = 3;
     }
     break;
   case 1: //RUNNING
     print_temp_humidity_lcd();
     print_water_level_serial();
     digitalWrite(blue_led, HIGH);
     digitalWrite(motor_pin, HIGH);
     if((temperature * 1.8 + 32) < 67){
       state = 0; 
       print_timestamp_serial();
       Serial.println("Motor off");
     }
     if(water_level < 100){
       state = 3;
       print_timestamp_serial();
       Serial.println("Motor off");
     }
     break;
   case 2: //DISABLED
     digitalWrite(yellow_led, HIGH);
     digitalWrite(motor_pin, LOW);
     break;
   case 3: //ERROR
     print_water_error_lcd();
     print_water_level_serial();
     digitalWrite(red_led, HIGH);
     digitalWrite(motor_pin, LOW);
     
     if (water_level > 100){
       state = 0;
     }
     break;
   default:
     Serial.print("Error, unknown state");
     state = 2; //DISABLED
     break;
 }
}


//PRINT FUNCTIONS
void print_timestamp_serial() {
  dt = clock.getDateTime();

  Serial.println("Motor Change: ");
  Serial.print(dt.year);   Serial.print("-");
  Serial.print(dt.month);  Serial.print("-");
  Serial.print(dt.day);    Serial.print(" ");
  Serial.print(dt.hour);   Serial.print(":");
  Serial.print(dt.minute); Serial.print(":");
  Serial.print(dt.second); Serial.println("");
}

void print_water_error_lcd() {
  lcd.setCursor (0, 0);
  lcd.clear();
  lcd.setCursor (1,0);
  lcd.write("Water level is");
  lcd.setCursor(5,1);
  lcd.write("low!");
}

void print_temp_humidity_lcd() {
  lcd.setCursor (0, 0);
  lcd.clear ();
  lcd.write("Temp:  ");
  lcd.print(temperature * 1.8 + 32);
  lcd.write("F  ");
  lcd.setCursor (0, 1);
  lcd.write("Humidity: ");
  lcd.print(humidity);
  lcd.write("%");
}

void print_water_level_serial() {
  Serial.print("W=");
  Serial.println(water_level, 1);
}

void print_temp_humidity_serial() {
  Serial.print("T = ");
  Serial.print(temperature * 1.8 + 32, 1);
  Serial.print(", H = ");
  Serial.println(humidity, 1);
}


//MISCELLANEOUS FUNCTIONS
void poll_vent() {
  vent_input_data = adc_read(vent_control_channel); //read from ADC channel
  vent_angle = vent_input_data * 0.18f; //Input ranges [0, 1000] -> Servo ranges [0, 180] conversion
  vent.write(vent_angle); //Servo library function
}


//ANALOG TO DIGITAL FUNCTIONS
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
