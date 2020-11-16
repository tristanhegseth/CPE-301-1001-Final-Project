/*
 * CPE 301 Final Project - Temperature and Humidity Test Code
 * Written by: Tristan Hegseth
 * November 16, 2020
 * 
 * Temperature and Humidity are read from DHT sensor 
 */

//TEMPERATURE HUMIDITY GLOBALS
#include <dht_nonblocking.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11

static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

float temperature = 0;
float humidity = 0;


//MAIN FUNCTIONS
void setup() {
  Serial.begin(9600);
}

void loop() {
  dht_sensor.measure(&temperature, &humidity);
  Serial.print( "T = " );
  Serial.print( temperature, 1 );
  Serial.print( " deg. C, H = " );
  Serial.print( humidity, 1 );
  Serial.println( "%" );
}
