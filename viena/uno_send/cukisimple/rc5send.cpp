#include<Arduino.h>

#define LED 9
#define BUTTON0 0
#define BUTTON1 1

#define FREQ 36000
#define DEVID 0x17

// Commands
#define COM0 13
#define COM1 42

bool TB;
bool Bstate[2];
int mS=0;

inline void sendSignal(uint16_t);

void setup()
{
	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);
	pinMode(BUTTON0, INPUT);
	pinMode(BUTTON1, INPUT);
}

void loop()
{
	if(Bstate[0])
	{
		if(!digitalRead(BUTTON0))  // Trqbva da pitam mi6o dali invert-va vhoda
		{
			Bstate[0]=false;
			TB^=1;
		}
		else sendSignal(COM0);
	}
	else if(Bstate[1])
	{
		if(!digitalRead(BUTTON1))  // Trqbva da pitam mi6o dali invert-va vhoda
		{
			Bstate[1]=false;
			TB^=1;
		}
		else sendSignal(COM1);
	}
	else
	{
		if(digitalRead(BUTTON0))  // Trqbva da pitam mi6o dali invert-va vhoda
		{
			Bstate[0]=true;
		}
		else if(digitalRead(BUTTON1))  // Trqbva da pitam mi6o dali invert-va vhoda
		{
			Bstate[1]=true;
		}
	}
}

inline void sendSignal(uint16_t Signal)
{
	if(millis()<mS)return; // Must not send signal now
	Signal|=(((6+TB)<<11)|(DEVID<<6)); // RC5 raboti
	for(int i=13;i>=0;--i)
	{
		if((Signal>>i)&1)
			delayMicroseconds(32000000/FREQ);
		for(int j=0;j<32;++j)
		{
			digitalWrite(LED, HIGH);
			delayMicroseconds(7); // Must be changed
			digitalWrite(LED, LOW);
			delayMicroseconds(21); // Must be changed
		}
		if((~Signal>>i)&1)
			delayMicroseconds(32000000/FREQ);
	}
	mS=millis()+4096000/FREQ; // Must not send signal for some time
}
