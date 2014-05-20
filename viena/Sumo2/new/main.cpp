#define F_CPU 32000000UL

#include<avr/interrupt.h>
#include<avr/eeprom.h>

#include "basic.h"
#include "motors.h"
#include "rc5.h"

#define STATE_POWER 0
#define STATE_FIGHT 1
#define STATE_DIEEE 2
#define ADDR_STATE ((uint8_t*)11)
#define ADDR_DOHYO ((uint8_t*)42)
#define SENSOR_SIDE_LEFT 2
#define SENSOR_FRONT_LEFT 3
#define SENSOR_FRONT_CENTER 4
#define SENSOR_FRONT_RIGHT 5
#define SENSOR_SIDE_RIGHT 6

volatile uint8_t State_Current;
volatile uint8_t Dohyo;
volatile int8_t blinktimes=0;

volatile bool EyeSensors[8];
volatile int8_t rotation=42;
inline void ReadSensors()
{
	for(uint8_t i=2;i<7;++i)
		EyeSensors[i]=IO_GET(PORTC,i);
}

int main()
{
	setClockTo32MHz();

	// Init LEDs
	IO_OUT(PORTD,4);IO_OUT(PORTD,5);
	IO_CLR(PORTD,4);IO_CLR(PORTD,5);

	// Read EEPROM
	State_Current=eeprom_read_byte(ADDR_STATE);
	if(State_Current!=STATE_FIGHT)State_Current=STATE_POWER;
	Dohyo=eeprom_read_byte(ADDR_DOHYO);

	// Set interrupts
	cli();
	RC5_init();
	MotorsInit();

	PORTD.PIN0CTRL=PORT_ISC_FALLING_gc; // Change PIN if needed
	PORTD.INT1MASK=1<<0; // Change PIN if needed
	PORTD.INTCTRL|=PMIC_LOLVLEN_bm<<2; // Eeprom clear button

	INT_TIMER(TCD0,32000,TC_CLKSEL_DIV1_gc,PMIC_LOLVLEN_bm)
	PMIC.CTRL|=7;sei(); // Enable interrupts

	while(1); // Good luck !!
}

ISR(TCC1_OVF_vect) // Do we have to blink?
{
	if(blinktimes-->0)
	{
		if(blinktimes&1)
			IO_SET(PORTD,4);
		else IO_CLR(PORTD,4);
	}
	else IO_TGL(PORTD,4);
	if(!blinktimes)
		TCC1.INTCTRLA=0; // Stop blinking
}
ISR(PORTD_INT1_vect) // Kill switch
{
	eeprom_write_byte(ADDR_STATE,42);
	State_Current=STATE_DIEEE;
}
ISR(PORTD_INT0_vect)
{
	IO_TGL(PORTD,5);
	if(RC5_timed)
	{
		RC5_timed=false;
		TCC0.CNT=0;
		RC5_buffer<<=1;
		RC5_buffer|=IO_GET(PORTD,3);
	}
	//else if(TCC0.CNT<10042)RC5_buffer=0;
	if(State_Current!=STATE_DIEEE&&RC5_start==3) // Read RC5
	{
		if(RC5_address==0x0B) // We need to reprogram
		{
			if(State_Current==STATE_POWER)
			{
				Dohyo=RC5_command;
				eeprom_write_byte(ADDR_DOHYO,Dohyo);
				// Blink two times fast
				if(!blinktimes)
				{
					blinktimes=4;
					INT_TIMER(TCC1,32000,TC_CLKSEL_DIV256_gc,PMIC_LOLVLEN_bm)
				}
			}
		}
		else if(RC5_address==0x07)
		{
			if(RC5_command==(Dohyo|1)) // We need to start
			{
				if(State_Current==STATE_POWER)
				{
					State_Current=STATE_FIGHT;
					eeprom_write_byte(ADDR_STATE,STATE_FIGHT);
					// Start LED while fighting
				}
			}
			else if(RC5_command==(Dohyo&~1)) // We need to stop
			{
				if(State_Current==STATE_FIGHT)
				{
					State_Current=STATE_DIEEE;
					eeprom_write_byte(ADDR_STATE,STATE_DIEEE);
					// Slow blink
					blinktimes=-1;
					INT_TIMER(TCC1,32000,TC_CLKSEL_DIV1024_gc,PMIC_LOLVLEN_bm)
				}
			}
		}
		// else Report a WTF! condition
	}
}

ISR(TCD0_OVF_vect) // Main Loop
{
	if(State_Current==STATE_FIGHT)
	{
		IO_SET(PORTD,4);
		//EnableMotors();
		ReadSensors();
//		if(EyeSensors[SENSOR_SIDE_LEFT])
//		{
//			rotation=-1;
//			setMotors(0,100);
//		}
//		else if(EyeSensors[SENSOR_SIDE_RIGHT])
//		{
//			rotation=1;
//			setMotors(100,0);
//		}
//		else if(EyeSensors[SENSOR_FRONT_LEFT]&&EyeSensors[SENSOR_FRONT_RIGHT])
//		{
//			if(EyeSensors[SENSOR_FRONT_CENTER])setMotors(60,60);
//			else setMotors(100,100);
//		}
//		else if(EyeSensors[SENSOR_FRONT_LEFT])
//		{
//			if(EyeSensors[SENSOR_FRONT_CENTER])setMotors(40,60);
//			else setMotors(40,80);
//		}
//		else if(EyeSensors[SENSOR_FRONT_RIGHT])
//		{
//			if(EyeSensors[SENSOR_FRONT_CENTER])setMotors(60,40);
//			else setMotors(80,40);
//		}
//		else if(rotation<0)
//			setMotors(0,100);
//		else setMotors(100,0);
	}
	else DisableMotors();
}
