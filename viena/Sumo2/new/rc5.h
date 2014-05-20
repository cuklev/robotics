#ifndef _RC5_H
#define _RC5_H

#define RC5_start ((RC5_buffer>>12)&3)
#define RC5_toggle ((RC5_buffer>>11)&1)
#define RC5_address ((RC5_buffer>>6)&31)
#define RC5_command (RC5_buffer&63)

volatile uint16_t RC5_buffer=0;
volatile bool RC5_timed=true;

inline void RC5_init()
{
	PORTD.PIN3CTRL=PORT_ISC_BOTHEDGES_gc; // Change PIN if needed
	PORTD.INT0MASK=1<<3; // Change PIN if needed
	PORTD.INTCTRL|=PMIC_LOLVLEN_bm;

	INT_TIMER(TCC0,40421,TC_CLKSEL_DIV1_gc,PMIC_LOLVLEN_bm)
}

ISR(TCC0_OVF_vect)
{
	RC5_timed=true;
}

#endif
