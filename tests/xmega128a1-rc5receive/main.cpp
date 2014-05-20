#define F_CPU 32000000UL

#include<avr/interrupt.h>
#include<util/delay.h>

void setClockTo32MHz();
void init();

volatile uint8_t gadost_mode=0;
volatile uint8_t mydevice=11;
struct ___cuki___{
	volatile uint16_t buffer;
	volatile uint8_t timed;
	inline volatile void read()
	{
		buffer=(buffer<<1)&((1<<14)-1); // mi6o pak e leko zaspal
		buffer|=(~PORTC.IN&1);
	}
	inline volatile uint8_t start(){return buffer>>12;}
	inline volatile uint8_t toggle(){return (buffer>>11)&1;}
	inline volatile uint8_t device(){return (buffer>>6)&31;}
	inline volatile uint8_t command(){return buffer&63;}
}rc5;

int main()
{
	setClockTo32MHz();
	init();
	while(1);
}

void setClockTo32MHz()
{
	CCP=CCP_IOREG_gc;			// disable register security for oscillator update
	OSC.CTRL=OSC_RC32MEN_bm;		// enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));	// wait for oscillator to be ready
	CCP=CCP_IOREG_gc;			// disable register security for clock update
	CLK.CTRL=CLK_SCLKSEL_RC32M_gc;		// switch to 32MHz clock
}

void init()
{
	PORTC.DIR=0x00; // сензор PORTC0
	PORTD.DIR=0x00; // бутончета
	PORTR.DIR=0x00; // бутончета
	PORTE.DIR=0xFF; // светодиоди
	PORTE.OUT=0xFF;

	//PORTC.PIN0CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_BOTHEDGES_gc; // Kaji na mi6o da sloji pullup
	PORTC.PIN0CTRL=PORT_ISC_BOTHEDGES_gc; // Kaji na mi6o da sloji pullup
	PORTD.PIN0CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTD.PIN1CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTD.PIN2CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTD.PIN3CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTD.PIN4CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTD.PIN5CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTR.PIN0CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTR.PIN1CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!

	cli();

	PORTC.INT0MASK=1;
	PORTC.INTCTRL=PMIC_LOLVLEN_bm;

	PORTD.INT0MASK=0x3F;
	PORTD.INTCTRL=PMIC_LOLVLEN_bm;
	PORTR.INT0MASK=3;
	PORTR.INTCTRL=1;

	TCF0.CTRLA=TC_CLKSEL_DIV1_gc;
	TCF0.CTRLB=TC_WGMODE_NORMAL_gc;
	TCF0.PER=40421;
	TCF0.CNT=0;
	TCF0.INTCTRLA=PMIC_LOLVLEN_bm;

	TCF1.CTRLA=TC_CLKSEL_DIV256_gc;
	TCF1.CTRLB=TC_WGMODE_NORMAL_gc;
	TCF1.PER=10000;
	TCF1.CNT=0;
	TCF1.INTCTRLA=PMIC_MEDLVLEN_bm;

	PMIC.CTRL|=7;
	sei();
}

ISR(TCF0_OVF_vect)
{
	rc5.timed=1;
}
ISR(TCF1_OVF_vect)
{
	if(gadost_mode==1)PORTE.OUT=(PORTE.OUT&0x80)|~rc5.command();
	else if(gadost_mode==2)PORTE.OUT=(PORTE.OUT&0x80)|~rc5.device();
	else if(rc5.start()==3)
	{
		if(rc5.device()==mydevice)
		{
			if(rc5.command()==17)PORTE.OUT=0b11111110;
			else if(rc5.command()==18)PORTE.OUT=0b11111101;
			else PORTE.OUT=0b11111011;
		}
		else PORTE.OUT=0b11110111;
	}
}

ISR(PORTC_INT0_vect)
{
	PORTE.OUT^=0x80;
	if(rc5.timed)
	{
		rc5.timed=0;
		TCF0.CNT=0;
		rc5.read();
	}
}

ISR(PORTR_INT0_vect)
{
	if(~PORTR.IN&1)gadost_mode=1;
	else if(~PORTR.IN&2)gadost_mode=2;
}
ISR(PORTD_INT0_vect)
{
	gadost_mode=0;
}
