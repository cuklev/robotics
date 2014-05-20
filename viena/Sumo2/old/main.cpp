#define F_CPU 32000000UL

#include<avr/interrupt.h>
#include<avr/eeprom.h>

#define P_IR PORTD    // PORTD3
#define P_LEDs PORTD  // PORTD4 PORTD5
#define P_SENSORS PORTC // ne znam 06te
#define P_BUTTON PORTD // PORTD0
#define P_MOTOR_L PORTR // PORTR
#define P_MOTOR_R PORTE // PORTE

#define TC_RC5TIMER TCC0
#define TC_BLINKTIMER TCC1
#define TC_MAIN TCD0
#define TC_PWM TCD1

#define INT_IR PORTD_INT0_vect
#define INT_RC5TIMER TCC0_OVF_vect
#define INT_BLINKTIMER TCC1_OVF_vect
#define INT_BUTTON PORTD_INT1_vect
#define INT_MAIN TCD0_OVF_vect
#define INT_PWM TCD1_OVF_vect

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
	P_IR.PIN3CTRL=PORT_ISC_BOTHEDGES_gc; // Change PIN if needed
	P_IR.INT0MASK=1<<3; // Change PIN if needed
	P_IR.INTCTRL=PMIC_LOLVLEN_bm;

	P_IR.PIN0CTRL=PORT_ISC_FALLING_gc; // Change PIN if needed
	P_IR.INT1MASK=1<<0; // Change PIN if needed
	P_IR.INTCTRL|=PMIC_LOLVLEN_bm<<2; // malka boza

	TC_RC5TIMER.CTRLB=TC_WGMODE_NORMAL_gc;
	TC_RC5TIMER.PER=40421;
	TC_RC5TIMER.CNT=0;
	TC_RC5TIMER.INTCTRLA=PMIC_LOLVLEN_bm;
	TC_RC5TIMER.CTRLA=TC_CLKSEL_DIV1_gc;
}

#define STATE_POWER 0
#define STATE_FIGHT 1
#define STATE_DIEEE 2
volatile uint8_t State_Current;

#define ADDR_STATE ((uint8_t*)11)
#define ADDR_DOHYO ((uint8_t*)42)

volatile uint16_t MotorLRate=0;
volatile uint16_t MotorRRate=0;

inline void MotorsInit()
{
	P_MOTOR_L.DIR|=0x0F; // Change PIN if needed
	P_MOTOR_R.DIR|=0x0F; // Change PIN if needed
	PORTE.OUT|=3;

	TC_PWM.CTRLB=TC_WGMODE_NORMAL_gc;
	TC_PWM.PER=64000;
	TC_PWM.CNT=0;
	TC_PWM.INTCTRLA=PMIC_LOLVLEN_bm;
	TC_PWM.CTRLA=TC_CLKSEL_DIV8_gc;
}
inline void setMotors(const uint16_t &_L,const uint16_t &_R)
{
	MotorLRate=_L*640;
	MotorRRate=_R*640;
}

#define SENSOR_SIDE_LEFT 2
#define SENSOR_FRONT_LEFT 3
#define SENSOR_FRONT_CENTER 4
#define SENSOR_FRONT_RIGHT 5
#define SENSOR_SIDE_RIGHT 6
volatile bool EyeSensors[8];
inline void ReadSensors()
{
	EyeSensors[SENSOR_SIDE_LEFT]=(P_SENSORS.IN>>SENSOR_SIDE_LEFT)&1;
	EyeSensors[SENSOR_FRONT_LEFT]=(P_SENSORS.IN>>SENSOR_FRONT_LEFT)&1;
	EyeSensors[SENSOR_FRONT_CENTER]=(P_SENSORS.IN>>SENSOR_FRONT_CENTER)&1;
	EyeSensors[SENSOR_FRONT_RIGHT]=(P_SENSORS.IN>>SENSOR_FRONT_RIGHT)&1;
	EyeSensors[SENSOR_SIDE_RIGHT]=(P_SENSORS.IN>>SENSOR_SIDE_RIGHT)&1;
}
volatile int8_t rotation=42;

int main()
{
	setClockTo32MHz();

	P_LEDs.DIR|=0x30; // светодиоди
	// Init LEDs
	P_LEDs.OUT=0x00; // Turn them off!

	// Read EEPROM
	State_Current=eeprom_read_byte(ADDR_STATE);
	if(State_Current!=STATE_FIGHT)State_Current=STATE_POWER;
	else P_LEDs.OUT|=0x10; // abe na vsqka stapka za posle
	Dohyo=eeprom_read_byte(ADDR_DOHYO);

	// Set interrupts
	cli();
	RC5_init();
	MotorsInit();
	TC_MAIN.CTRLB=TC_WGMODE_NORMAL_gc;
	TC_MAIN.PER=32000;
	TC_MAIN.CNT=0;
	TC_MAIN.INTCTRLA=PMIC_LOLVLEN_bm;
	TC_MAIN.CTRLA=TC_CLKSEL_DIV1_gc;
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
	P_LEDs.OUT^=0x20;
	if(RC5_timed)
	{
		RC5_timed=false;
		TC_RC5TIMER.CNT=0;
		RC5_buffer<<=1;
		RC5_buffer|=((~P_IR.IN>>3)&1); // Change PIN if needed
	}
	else if(TC_RC5TIMER.CNT<10042)RC5_buffer=0;
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
					TC_BLINKTIMER.CTRLB=TC_WGMODE_NORMAL_gc;
					TC_BLINKTIMER.PER=32000;
					TC_BLINKTIMER.CNT=0;
					TC_BLINKTIMER.INTCTRLA=PMIC_LOLVLEN_bm;
					TC_BLINKTIMER.CTRLA=TC_CLKSEL_DIV256_gc;
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
					P_LEDs.OUT|=0x10; // abe na vsqka stapka za posle
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
					TC_BLINKTIMER.CTRLB=TC_WGMODE_NORMAL_gc;
					TC_BLINKTIMER.PER=32000;
					TC_BLINKTIMER.CNT=0;
					TC_BLINKTIMER.INTCTRLA=PMIC_LOLVLEN_bm;
					TC_BLINKTIMER.CTRLA=TC_CLKSEL_DIV1024_gc;
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
	if(blinktimes&1)
		P_LEDs.OUT|=0x10;
	else P_LEDs.OUT&=~0x10;
	if(!blinktimes)
		TC_BLINKTIMER.INTCTRLA=0;
}
ISR(INT_BUTTON)
{
	eeprom_write_byte(ADDR_STATE,42);
}
ISR(INT_PWM)
{
	if(TC_PWM.CNT<MotorLRate)
	{
		P_MOTOR_L.OUT|=0x01; // Change PIN if needed
	}
	else
	{
		P_MOTOR_L.OUT&=~0x01; // Change PIN if needed
	}
	if(TC_PWM.CNT<MotorRRate)
	{
		P_MOTOR_R.OUT|=0x01; // Change PIN if needed
	}
	else
	{
		P_MOTOR_R.OUT&=~0x01; // Change PIN if needed
	}
}
ISR(INT_MAIN)
{
	if(State_Current==STATE_FIGHT)
	{
		P_LEDs.OUT|=0x10;
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
	else setMotors(0,0);
}
