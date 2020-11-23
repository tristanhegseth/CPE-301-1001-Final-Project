
const int sensorPin = A0;
int HistoryValue=0;
char printBuffer[25];


void setup() {
  Serial.begin(9600);


}

void loop() {
  int value = analogRead(sensorPin);//et adc value 
  int diffValue = HistoryValue - value;
  //
if((abs(diffValue)>=10 ) || ((value == 0) &&(HistoryValue !=0)))//
  {
    sprintf(printBuffer, "Water level is %04d\n", value);
    Serial.print(printBuffer);
    HistoryValue = value;
  }
}
