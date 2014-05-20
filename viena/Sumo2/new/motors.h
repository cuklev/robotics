#ifndef _MOTORS_H
#define _MOTORS_H

#include "basic.h"

volatile uint16_t MotorLRate=0;
volatile uint16_t MotorRRate=0;

inline void DisableMotors()
{
	IO_CLR(PORTE,2);
	IO_CLR(PORTE,3);
}
inline void EnableMotors()
{
	IO_SET(PORTE,2);
	IO_SET(PORTE,3);
}

inline void MotorsInit()
{
	IO_OUT(PORTE,0);IO_OUT(PORTE,1);
	IO_OUT(PORTE,2);IO_OUT(PORTE,3); // Left motor control
	IO_OUT(PORTR,2);IO_OUT(PORTR,3); // Right motor control

	INT_TIMER(TCD1,64000,TC_CLKSEL_DIV8_gc,PMIC_LOLVLEN_bm) // PWM
}

inline void setMotors(const int16_t &_L,const int16_t &_R)
{
	MotorLRate=_L*640;
	MotorRRate=_R*640;
}

ISR(TCD1_OVF_vect)
{
	if(TCD1.CNT<MotorLRate)
	{
		IO_SET(PORTR,2);
		IO_CLR(PORTR,3);
	}
	else
	{
		IO_CLR(PORTR,2);
		IO_SET(PORTR,3);
	}
	if(TCD1.CNT<MotorRRate)
	{
		IO_SET(PORTE,2);
		IO_CLR(PORTE,3);
	}
	else
	{
		IO_CLR(PORTE,2);
		IO_SET(PORTE,3);
	}
}

#endif
