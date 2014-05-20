#include<Arduino.h>

void setup()
{
	pinMode(13, OUTPUT) ;
	digitalWrite(13, LOW) ;
}

void loop()
{
	delay(1000);
	digitalWrite(13,HIGH);
	delay(1000);
	digitalWrite(13,LOW);
}
