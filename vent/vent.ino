/*
 * CPE 301 Final Project - Vent Angle Test Code
 * Written by: Tristan Hegseth 
 * November 16, 2020
 * 
 * Takes analog input from a potentiometer based voltage divider to control the angle of a servo motor
 */

//VENT GLOBALS
#include <Servo.h>
Servo vent; //Vent angle servo motor

int vent_motor_pin = 9;
int vent_control_pin = A0;    // select the input pin for the potentiometer
int vent_control_channel = vent_control_pin - 54; //only on ATmega2560
int vent_input_data = 0;
float vent_angle = 0;    // variable to store the servo position


//MAIN FUNCTIONS
void setup() {
  adc_init(); //Enable Analog to Digital Read
  vent.attach(vent_motor_pin); //Servo pin initialization
}
void loop() {
  vent_input_data = adc_read(vent_control_channel); //read from ADC channel
  vent_angle = vent_input_data / 1000.f * 180.f; //Input ranges [0, 1000] -> Servo ranges [0, 180] conversion
  vent.write(vent_angle); 
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
