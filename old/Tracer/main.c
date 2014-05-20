/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#define MAXMAXSPEED 25
#define USER_LED0    IOPORT_CREATE_PIN(PORTD, 5)
#define USER_LED1    IOPORT_CREATE_PIN(PORTD, 4)
#define BUTTON    IOPORT_CREATE_PIN(PORTD, 0)
#define F_CPU 32000000UL
#include <asf.h>
#include "util/delay.h"

#define MY_ADC    ADCA
#define MY_ADC_CH ADC_CH0

int MAXSPEED = MAXMAXSPEED;

struct pwm_config pwm_cfg[2];
int sensorsMax[12];
int sensorsMin[12];
int lastState;
int lastCorrctStat;

float 
	K = 1,
	ISum = 0,
	Kp,
	Ki,
	Kd;
int lastValue;

int computePID(int setPoint, int realValue);
static void adc_init(void);
void set_motors(int speed,int turn);
void set_motorA(int dir, int speed);
void set_motorB(int dir, int speed);
void init_motors(void);
uint8_t read_sensor(int sensor);
uint16_t read_adc(int port);
void tc_init(void);
void mainLoop(void);
uint8_t read_sensor(int sensor);
void sensors_init(void);
int sensors_read(void);

int main (void)
{
	Kp = 1 * K;
	Ki = 0 * K;
	Kd = 0 * K;
	board_init();
	sysclk_init();
	pmic_init();
	adc_init();
	init_motors();
	sensors_init();
	tc_init();
	
	PORTD.DIRSET = 0xFF;
	ioport_set_pin_dir(USER_LED0, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(USER_LED1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(BUTTON, IOPORT_DIR_INPUT);
	PORTE.DIRSET = 0xFF;
	PORTR.DIRSET = 0xFF;
	
	
	
	while (1)
	{
	}
}
int motorComand;
int buttonPressed = 0;
void mainLoop()
{
	if (ioport_get_pin_level(BUTTON) == 0 && buttonPressed == 0)
	{
		buttonPressed = 1;
		_delay_ms(1000);
		for (int i = 0; i < 100; ++i) {
			set_motors(i, 0);
			_delay_ms(2);
		}
		set_motors(100, 0);
		_delay_ms(2000);
		set_motors(90, 150);
		_delay_ms(500);
		set_motors(0,0);
		_delay_ms(20000);
	}
	int linePosition = sensors_read();
	if (linePosition != -13)
	{
		if (buttonPressed == 1)
		{
			motorComand = computePID(0,linePosition);
			MAXSPEED = (MAXMAXSPEED - abs(linePosition)/12 * MAXMAXSPEED);
			if (MAXSPEED == 0)
			{
				MAXSPEED = 15;
			}
			lastCorrctStat = linePosition;
			set_motors(MAXSPEED, motorComand);
		}
		ioport_set_pin_level(USER_LED0, 0);
	} 
	else
	{
		if (buttonPressed == 1)
		{
			motorComand = computePID(0, lastCorrctStat);
			set_motors(MAXSPEED, motorComand);
		}
		ioport_set_pin_level(USER_LED0, 1);
	}
}

int lasterror = 0;
int computePID(int setPoint, int realValue)
{
	int error = setPoint - realValue;
	float PValue = error * Kp;
	ISum += error * Ki;
	float DValue = (error - lasterror) * Kd;
	float result = (PValue + ISum - DValue) * 25;
	lasterror = error;
	if (result > 300)
	{
		result = 300;
	}
	if (result < -300)
	{
		result = -300;
	}
	return result;
}

void tc_init()
{
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0, mainLoop);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);
	tc_write_period(&TCC0,32000); // 1KHz main loop
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_LO);
	cpu_irq_enable();
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV1_gc);

}

void init_motors()
{
	pwm_init(&pwm_cfg[0], PWM_TCE0, PWM_CH_A, 200); /* PE0 */
	pwm_init(&pwm_cfg[1], PWM_TCE0, PWM_CH_B, 200); /* PE1 */
}

void set_motors(int speed,int turn)
{
	int speed_A=speed,speed_B=speed,dir_A=1,dir_B=1;
	if (turn >= 0)
	{
		/*
		if (turn <= 100)
		{
			speed_A = speed;
			speed_B = speed + turn / MAXMAXSPEED * (MAXMAXSPEED - speed);
			dir_A = 1;
			dir_B = 1;
		} 
		else
		*/
		 if (turn <= 100)
		{
			//turn -= 100;
			speed_A = speed - speed * turn / 100;
			speed_B = speed;
			dir_A = 1;
			dir_B = 1;
		}
		else if(turn <= 200)
		{
			turn -= 100;
			speed_A = speed * turn / 100;
			speed_B = speed;
			dir_A = 0;
			dir_B = 1;
		}
		else
		{
			turn -= 200;
			speed_A = speed * turn / 100;
			speed_B = speed;
			dir_A = -1;
			dir_B = 1;
		}
	}
	else
	{
		/*
		if (-100 <= turn)
		{
			speed_A = speed - turn / MAXMAXSPEED * (MAXMAXSPEED - speed);
			speed_B = speed;
			dir_A = 1;
			dir_B = 1;
		} 
		else
		*/ 
		if (-100 <= turn)
		{
			//turn += 100;
			speed_A = speed;
			speed_B = speed + speed * turn / 100;
			dir_A = 1;
			dir_B = 1;
		}
		else if (-200 <= turn)
		{
			turn += 100;
			speed_A = speed;
			speed_B = - speed * turn / 100;
			dir_A = 1;
			dir_B = 0;
		}
		else
		{
			turn += 200;
			speed_A = speed;
			speed_B = - speed * turn / 100;
			dir_A = 1;
			dir_B = -1;
		}
	}
	set_motorA(dir_A,speed_A);
	set_motorB(dir_B,speed_B);
}

void set_motorB(int dir, int speed)
{
	if (dir == -1)
	{
		PORTE.OUT |= PIN2_bm;
		PORTE.OUT &= ~PIN3_bm;
	}
	else if(dir == 1)
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

uint16_t read_adc(int port)
{
	struct adc_channel_config adcch_conf;
	uint16_t result;
	
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
	
	
	adcch_set_input(&adcch_conf, port, ADCCH_NEG_INTERNAL_GND, 1);
	
	adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
	
	adc_enable(&MY_ADC);

	adc_start_conversion(&MY_ADC, MY_ADC_CH);
	adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH);

	result = adc_get_result(&MY_ADC, MY_ADC_CH);
	
	return result;
}

static void adc_init(void)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;

	adc_read_configuration(&MY_ADC, &adc_conf);
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);

	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12,ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 2000000UL);

	adcch_set_input(&adcch_conf, ADCCH_POS_PIN6, ADCCH_NEG_PIN4,1);

	adc_write_configuration(&MY_ADC, &adc_conf);
	adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
	
	read_adc(0);
}
uint8_t read_sensor(int sensor)
{
	read_adc(sensor);
	int read_value = read_adc(sensor);
	if (read_value > sensorsMax[sensor])
	{
		sensorsMax[sensor] = read_value;
	}
	if (read_value < sensorsMin[sensor])
	{
		sensorsMin[sensor] = read_value;
	}
	int middleVal = (sensorsMax[sensor] + sensorsMin[sensor])/2;
	if (read_value > middleVal)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void sensors_init()
{
	for (int i=0;i<12;i++)
	{
		sensorsMax[i] = 0;
		sensorsMin[i] = 4000;
	}
}

int sensors_read()
{
	int sensors_states[12];
	for (int i=0;i<12;i++)
	{
		sensors_states[i] = read_sensor(i);
	}
	int possible_states[][12] =
	{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0},
		{0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}
	};
	int state = -1,flag = 1,j = 0;
	while (state == -1 && j < 24)
	{
		flag = 1;
		for (int i=0;i<12;i++)
		{
			if (possible_states[j][i] != sensors_states[i])
			{
				flag = 0;
			}
		}
		if (flag)
		{
			state = j;
		}
		else
		{
			j++;
		}
	}
	if (state == 0 && lastState == 23)
	{
		state = 24;
	}
	if (state == 0 && lastState == 12)
	{
		state = 12;
	}
	if (state == 0 && lastState == 24)
	{
		state = 24;
	}
	lastState = state;
	return state-12;
}