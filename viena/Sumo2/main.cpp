#define F_CPU 32000000UL

#include<avr/interrupt.h>
#include<avr/eeprom.h>
#include<util/delay.h>

#define IO_OUT(_PORT,_PIN) (_PORT.DIR|=(1<<_PIN))
#define IO_SET(_PORT,_PIN) (_PORT.OUT|=(1<<_PIN))
#define IO_CLR(_PORT,_PIN) (_PORT.OUT&=~(1<<_PIN))
#define IO_TGL(_PORT,_PIN) (_PORT.OUT^=(1<<_PIN))
#define IO_GET(_PORT,_PIN) ((_PORT.IN>>_PIN)&1)

#define INT_TIMER(_TC,_PER,_DIV,_LVL) \
	_TC.CTRLB=TC_WGMODE_NORMAL_gc; \
	_TC.PER=_PER; \
	_TC.CNT=0; \
	_TC.INTCTRLA=_LVL; \
	_TC.CTRLA=_DIV;

#define RC5_start ((RC5_buffer>>12)&3)
#define RC5_toggle ((RC5_buffer>>11)&1)
#define RC5_address ((RC5_buffer>>6)&31)
#define RC5_command (RC5_buffer&63)

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

inline void setClockTo32MHz()
{
	CCP=CCP_IOREG_gc;			// disable register security for oscillator update
	OSC.CTRL=OSC_RC32MEN_bm;		// enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));	// wait for oscillator to be ready
	CCP=CCP_IOREG_gc;			// disable register security for clock update
	CLK.CTRL=CLK_SCLKSEL_RC32M_gc;		// switch to 32MHz clock
}

volatile uint16_t RC5_buffer=0;
volatile bool RC5_timed=true;
volatile uint8_t State_Current;
volatile uint8_t Dohyo;
volatile int8_t blinktimes=0;
volatile int8_t rotation=42;
volatile bool EyeSensors[8];

inline void RC5_init()
{
	PORTD.PIN3CTRL=PORT_ISC_BOTHEDGES_gc; // Change PIN if needed
	PORTD.INT0MASK=1<<3; // Change PIN if needed
	PORTD.INTCTRL=PMIC_MEDLVLEN_bm; // vij beee

	INT_TIMER(TCC0,40421,TC_CLKSEL_DIV1_gc,PMIC_MEDLVLEN_bm)
}

inline void MotorsInit()
{
	IO_OUT(PORTE,0);IO_OUT(PORTE,1);
	
	IO_OUT(PORTR,0);IO_OUT(PORTR,1); // Left motor control
	IO_OUT(PORTE,2);IO_OUT(PORTE,3); // Right motor control
	
	IO_CLR(PORTE,2);IO_CLR(PORTE,3);
	IO_CLR(PORTR,0);IO_CLR(PORTR,1);	

	INT_TIMER(TCE0,64000,TC_CLKSEL_DIV8_gc,0) // PWM

	TCE0.CTRLB|=(TC0_CCAEN_bm|TC0_CCBEN_bm|TC_WGMODE_SS_gc); // MAGIC
	TCE0.CCA=0;TCE0.CCB=0;
}
inline void setMotors(const int16_t &_L,const int16_t &_R)
{
	if(_L<0)
	{
		IO_SET(PORTR,1);
		IO_CLR(PORTR,0);
	}
	else
	{
		IO_SET(PORTR,0);
		IO_CLR(PORTR,1);
	}
	if(_R<0)
	{
		IO_SET(PORTE,2);
		IO_CLR(PORTE,3);
	}
	else
	{
		IO_SET(PORTE,3);
		IO_CLR(PORTE,2);
	}
	TCE0.CCB=(_L<0)?-640*_L:640*_L;
	TCE0.CCA=(_R<0)?-640*_R:640*_R;
}

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

	// Define interrupts
	cli();

	RC5_init();
	MotorsInit();

	INT_TIMER(TCD0,32000,TC_CLKSEL_DIV1_gc,PMIC_LOLVLEN_bm) // Main Loop

	PORTD.DIR&=~3;

	PMIC.CTRL|=7;
	sei(); // Enable interrupts

	while(1); // Good luck !!
}

ISR(TCC0_OVF_vect)
{
	RC5_timed=true;
}
ISR(PORTD_INT0_vect)
{
	IO_TGL(PORTD,5);
	if(RC5_timed)
	{
		RC5_timed=false;
		TCC0.CNT=0;
		RC5_buffer=(RC5_buffer<<1)&((1<<14)-1);
		RC5_buffer|=!IO_GET(PORTD,3); // Change PIN if needed
	}
	else if(TCC0.CNT<10042)RC5_buffer=0;
	if(State_Current!=STATE_DIEEE&&RC5_start==3)
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
ISR(TCD0_OVF_vect) // Main Loop
{
	if(State_Current==STATE_FIGHT)
	{
		if(!IO_GET(PORTD,0)) // Kill switch
		{
			State_Current=STATE_DIEEE;
			eeprom_write_byte(ADDR_STATE,STATE_DIEEE);
			// Slow blink
			blinktimes=-1;
			INT_TIMER(TCC1,32000,TC_CLKSEL_DIV1024_gc,PMIC_LOLVLEN_bm)
		}
		IO_SET(PORTD,4);
		ReadSensors();
		if(EyeSensors[SENSOR_SIDE_LEFT])
		{
			rotation=-1;
			setMotors(0,100);
		}
		else if(EyeSensors[SENSOR_SIDE_RIGHT])
		{
			rotation=1;
			setMotors(100,0);
		}
		else if(EyeSensors[SENSOR_FRONT_LEFT]&&EyeSensors[SENSOR_FRONT_RIGHT])
		{
			if(EyeSensors[SENSOR_FRONT_CENTER])setMotors(60,60);
			else setMotors(100,100);
		}
		else if(EyeSensors[SENSOR_FRONT_LEFT])
		{
			if(EyeSensors[SENSOR_FRONT_CENTER])setMotors(40,60);
			else setMotors(40,80);
		}
		else if(EyeSensors[SENSOR_FRONT_RIGHT])
		{
			if(EyeSensors[SENSOR_FRONT_CENTER])setMotors(60,40);
			else setMotors(80,40);
		}
		else if(rotation<0)
			setMotors(0,100);
		else setMotors(100,0);
	}
	else setMotors(0,0);
}
