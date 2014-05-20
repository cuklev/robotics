#ifndef _BASIC_H
#define _BASIC_H

#include<avr/interrupt.h>

inline void setClockTo32MHz()
{
	CCP=CCP_IOREG_gc;			// disable register security for oscillator update
	OSC.CTRL=OSC_RC32MEN_bm;		// enable 32MHz oscillator
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));	// wait for oscillator to be ready
	CCP=CCP_IOREG_gc;			// disable register security for clock update
	CLK.CTRL=CLK_SCLKSEL_RC32M_gc;		// switch to 32MHz clock
}

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

#endif
