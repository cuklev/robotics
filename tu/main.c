#define F_CPU 32000000UL

#include<asf.h>
#include<util/delay.h>

#define LEDBK IOPORT_CREATE_PIN(PORTD, 4)
#define LEDFR IOPORT_CREATE_PIN(PORTD, 5)

#define BUTTON IOPORT_CREATE_PIN(PORTD, 0)

#define SENSOR_TOP IOPORT_CREATE_PIN(PORTC, 2)
#define SENSOR_FRL IOPORT_CREATE_PIN(PORTC, 3)
#define SENSOR_LLL IOPORT_CREATE_PIN(PORTC, 4)
#define SENSOR_FRR IOPORT_CREATE_PIN(PORTC, 5)
#define SENSOR_RRR IOPORT_CREATE_PIN(PORTC, 6)

#define MAXSPEED 100
#define INITSPEED 20
#define TURNSPEED 15
#define CORRECTSPEED 25

bool Sensor_top;
bool Sensor_frl;
bool Sensor_lll;
bool Sensor_frr;
bool Sensor_rrr;
bool Sensor_but;

int disable=0;
bool competing=false;
int rotation=42;

struct pwm_config pwm_cfg[2];

void init_motors(void);
void set_motorA(int dir, int speed);
void set_motorB(int dir, int speed);
void set_motors(int speed, int turn);
void readSensors(void);
void mainLoop(void);

int main(void)
{
	// Hardware initialisation
	board_init();
	sysclk_init();
	pmic_init();
	init_motors();

	ioport_set_pin_dir(LEDFR, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(LEDBK, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(BUTTON, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SENSOR_TOP, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SENSOR_FRL, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SENSOR_FRR, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SENSOR_LLL, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SENSOR_RRR, IOPORT_DIR_INPUT);

	// Wait for signal
	disable=40;
	while(1)
	{
		readSensors();
		ioport_set_pin_level(LEDFR, Sensor_frl || Sensor_frr);
		ioport_set_pin_level(LEDBK, Sensor_lll || Sensor_rrr);
		if(disable>0)--disable;
		else if(Sensor_top || Sensor_but)
		{
			disable=500;
			competing=true;
			break;
		}
		_delay_ms(25);
	}

	// Timer counter start
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0, mainLoop);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);
	tc_write_period(&TCC0, 32000); // 1kHz main loop
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_LO);
	cpu_irq_enable();
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV1_gc);

	set_motors(20, 0);
	while(1);
}

void readSensors(void)
{
	Sensor_top=!ioport_get_pin_level(SENSOR_TOP);
	Sensor_frl=!ioport_get_pin_level(SENSOR_FRL);
	Sensor_lll=!ioport_get_pin_level(SENSOR_LLL);
	Sensor_frr=!ioport_get_pin_level(SENSOR_FRR);
	Sensor_rrr=!ioport_get_pin_level(SENSOR_RRR);
	Sensor_but=!ioport_get_pin_level(BUTTON);
}

void mainLoop(void)
{
	readSensors();
	if(disable>0)--disable;
	else if(Sensor_but||Sensor_top)
	{
		tc_write_clock_source(&TCC0, 0);
		_delay_ms(100);
		readSensors();
		if(Sensor_but||Sensor_top)competing=false;
		else tc_write_clock_source(&TCC0, TC_CLKSEL_DIV1_gc);
	}
	if(!competing)
	{
		set_motors(20, 20);
		return;
	}
	
	// FIGHT!
	
	if(Sensor_frl && Sensor_frr)
		rotation=0;
	else if(Sensor_frl)rotation=-1;
	else if(Sensor_frr)rotation=1;
	else if(Sensor_lll)rotation=-2;
	else if(Sensor_rrr)rotation=2;
	
	if(rotation==0) set_motors(MAXSPEED, MAXSPEED);
	else if(rotation==1) set_motors(40, 30);
	else if(rotation==-1) set_motors(30, 40);
	else if(rotation==2) set_motors(40, 20);
	else if(rotation==-2) set_motors(20, 40);
}

void set_motors(int speedA, int speedB)
{
	int dirA,dirB;
	if(speedA<0)
	{
		dirA=-1;
		speedA*=-1;
	}
	else dirA=(speedA>0);
	if(speedB<0)
	{
		dirB=-1;
		speedB*=-1;
	}
	else dirB=(speedB>0);
	dirA*=-1; // 6toto mi6o e tup
	dirB*=-1; // 6toto mi6o e tup
	set_motorA(dirA,speedA);
	set_motorB(dirB,speedB);
}

void init_motors(void)
{
	PORTE.DIRSET = 0xFF;
	PORTR.DIRSET = 0xFF;
	pwm_init(&pwm_cfg[0], PWM_TCE0, PWM_CH_A, 50); /* PE0 */
	pwm_init(&pwm_cfg[1], PWM_TCE0, PWM_CH_B, 50); /* PE1 */
}

void set_motorA(int dir, int speed)
{
	if (dir == 1)
	{
		PORTR.OUT |= PIN0_bm;
		PORTR.OUT &= ~PIN1_bm;
	}
	else if(dir == -1)
	{
		PORTR.OUT &= ~PIN0_bm;
		PORTR.OUT |= PIN1_bm;
	}
	else
	{
		PORTR.OUT &= ~PIN0_bm;
		PORTR.OUT &= ~PIN1_bm;
	}
	pwm_start(&pwm_cfg[1], speed);
}

void set_motorB(int dir, int speed)
{
	if (dir == 1)
	{
		PORTE.OUT |= PIN2_bm;
		PORTE.OUT &= ~PIN3_bm;
	}
	else if(dir == -1)
	{
		PORTE.OUT &= ~PIN2_bm;
		PORTE.OUT |= PIN3_bm;
	}
	else
	{
		PORTE.OUT &= ~PIN2_bm;
		PORTE.OUT &= ~PIN3_bm;
	}
	pwm_start(&pwm_cfg[0], speed);
}
