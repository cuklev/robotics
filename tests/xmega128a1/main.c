#define F_CPU 32000000UL

#include<avr/interrupt.h>
#include<util/delay.h>

void setClockTo32MHz(void);
void init(void);

int main(void)
{
	setClockTo32MHz();
	init();
	while(1);
}

void setClockTo32MHz(void)
{
	CCP=CCP_IOREG_gc;			// disable register security for oscillator update
	OSC.CTRL=OSC_RC32MEN_bm;		// enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));	// wait for oscillator to be ready
	CCP=CCP_IOREG_gc;			// disable register security for clock update
	CLK.CTRL=CLK_SCLKSEL_RC32M_gc;		// switch to 32MHz clock
}

void init(void)
{
	PORTD.DIR=0x00;
	PORTR.DIR=0x00;
	PORTE.DIR=0xFF;

	PORTD.PIN0CTRL=PORT_OPC_PULLUP_gc;
	PORTD.PIN1CTRL=PORT_OPC_PULLUP_gc;
	PORTD.PIN2CTRL=PORT_OPC_PULLUP_gc;
	PORTD.PIN3CTRL=PORT_OPC_PULLUP_gc;
	PORTD.PIN4CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_RISING_gc;
	PORTD.PIN5CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc;
	PORTR.PIN0CTRL=PORT_OPC_PULLUP_gc;
	PORTR.PIN1CTRL=PORT_OPC_PULLUP_gc;

	cli();

	TCC0.CTRLA=TC_CLKSEL_DIV64_gc;
	TCC0.CTRLB=TC_WGMODE_NORMAL_gc;
	TCC0.PER=32000;
	TCC0.CNT=0;
	TCC0.INTCTRLA=1PMIC_LOLVLEN_bm;

	TCC1.CTRLA=TC_CLKSEL_DIV1024_gc;
	TCC1.CTRLB=TC_WGMODE_NORMAL_gc;
	TCC1.PER=5000;
	TCC1.CNT=0;
	TCC1.INTCTRLA=PMIC_LOLVLEN_bm;

	PORTD.INT0MASK=3<<4;
	PORTD.INTCTRL=1;
	PORTR.INT0MASK=1<<0;
	PORTR.INTCTRL=PMIC_LOLVLEN_bm;

	PMIC.CTRL|=7;
	sei();
}

ISR(TCC0_OVF_vect) 
{
	PORTE.OUTTGL=5;
}

ISR(TCC1_OVF_vect) 
{
	PORTE.OUTTGL=6;
}

ISR(PORTD_INT0_vect)
{
	PORTE.OUTTGL=15<<4;
}

ISR(PORTR_INT0_vect)
{
	PORTE.OUTTGL=~15;
}
