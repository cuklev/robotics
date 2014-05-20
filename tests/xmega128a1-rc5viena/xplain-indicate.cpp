#define F_CPU 32000000UL

#include<avr/interrupt.h>
#include<avr/eeprom.h>

#define P_IR PORTC
#define P_LEDs PORTE
#define P_SENSORS PORTF

#define TC_RC5TIMER TCF0
#define TC_BLINKTIMER TCF1

#define INT_IR PORTC_INT0_vect
#define INT_RC5TIMER TCF0_OVF_vect
#define INT_BLINKTIMER TCF1_OVF_vect

volatile uint8_t blinktimes=0;

inline void setClockTo32MHz()
{
	CCP=CCP_IOREG_gc;			// disable register security for oscillator update
	OSC.CTRL=OSC_RC32MEN_bm;		// enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));	// wait for oscillator to be ready
	CCP=CCP_IOREG_gc;			// disable register security for clock update
	CLK.CTRL=CLK_SCLKSEL_RC32M_gc;		// switch to 32MHz clock
}

#define RC5_start ((RC5_buffer>>12)&3)
#define RC5_toggle ((RC5_buffer>>11)&1)
#define RC5_address ((RC5_buffer>>6)&31)
#define RC5_command (RC5_buffer&63)
volatile uint16_t RC5_buffer=0;
volatile bool RC5_timed=true;
volatile uint8_t Dohyo;

inline void RC5_init()
{
	P_IR.DIR=0x00;
	P_IR.PIN0CTRL=PORT_ISC_BOTHEDGES_gc; // Change PIN if needed
	P_IR.INT0MASK=1; // Change PIN if needed
	P_IR.INTCTRL=PMIC_LOLVLEN_bm;

	TC_RC5TIMER.CTRLA=TC_CLKSEL_DIV1_gc;
	TC_RC5TIMER.CTRLB=TC_WGMODE_NORMAL_gc;
	TC_RC5TIMER.PER=40421;
	TC_RC5TIMER.CNT=0;
	TC_RC5TIMER.INTCTRLA=PMIC_LOLVLEN_bm;
}

#define STATE_POWER 0
#define STATE_FIGHT 1
#define STATE_DIEEE 2
volatile uint8_t State_Current;

#define ADDR_STATE ((uint8_t*)42)
#define ADDR_DOHYO ((uint8_t*)43)

int main()
{
	setClockTo32MHz();

	// Init LEDs
	P_LEDs.DIR=0xFF; // светодиоди
	P_LEDs.OUT=0xFF; // Turn them off!

	// Read EEPROM
	State_Current=eeprom_read_byte(ADDR_STATE);
	if(State_Current!=STATE_FIGHT)State_Current=STATE_POWER;
	else P_LEDs.OUT=0xF0; // abe na vsqka stapka za posle
	Dohyo=eeprom_read_byte(ADDR_DOHYO);

	// Set interrupts
	cli();
	RC5_init();
	PMIC.CTRL|=7;
	sei();

	while(1); // Good luck !!
}

ISR(INT_RC5TIMER)
{
	RC5_timed=true;
}
ISR(INT_IR)
{
	if(P_IR.IN&1)P_LEDs.OUTSET=0x80;else P_LEDs.OUTCLR=0x80; // Test signal
	if(RC5_timed)
	{
		RC5_timed=false;
		TC_RC5TIMER.CNT=0;
		RC5_buffer<<=1;
		RC5_buffer|=(~P_IR.IN&1); // Change PIN if needed
	}
	if(RC5_start==3)
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
					TC_BLINKTIMER.CTRLA=TC_CLKSEL_DIV256_gc;
					TC_BLINKTIMER.CTRLB=TC_WGMODE_NORMAL_gc;
					TC_BLINKTIMER.PER=32000;
					TC_BLINKTIMER.CNT=0;
					TC_BLINKTIMER.INTCTRLA=PMIC_LOLVLEN_bm;
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
					// Start LED while
					P_LEDs.OUT=0xF0; // abe na vsqka stapka za posle
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
					TC_BLINKTIMER.CTRLA=TC_CLKSEL_DIV1024_gc;
					TC_BLINKTIMER.CTRLB=TC_WGMODE_NORMAL_gc;
					TC_BLINKTIMER.PER=32000;
					TC_BLINKTIMER.CNT=0;
					TC_BLINKTIMER.INTCTRLA=PMIC_LOLVLEN_bm;
				}
			}
		}
		// else Report a WTF! condition
	}
}
ISR(INT_BLINKTIMER)
{
	if(blinktimes>0)
		--blinktimes;
	P_LEDs.OUT^=0x0F;
	if(!blinktimes)
		TC_BLINKTIMER.INTCTRLA=0;
}
