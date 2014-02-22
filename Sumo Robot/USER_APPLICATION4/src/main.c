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

#define F_CPU 32000000UL
#include <asf.h>
#include "util/delay.h"

#define DETECTVALUE 3600
#define USER_LED0    IOPORT_CREATE_PIN(PORTD, 5)
#define USER_LED1    IOPORT_CREATE_PIN(PORTD, 4)
#define MY_BUTTON    IOPORT_CREATE_PIN(PORTD, 0)
#define MAXSPEED 90

#define MY_ADC    ADCA
#define MY_ADC_CH ADC_CH0

struct pwm_config pwm_cfg[2];
int lastOponentPosition = 0;
int readings_L_SUM = 0;
int readings_R_SUM = 0;

static void adc_init(void);
void set_motors(int speed,int turn);
void set_motorA(int dir, int speed);
void set_motorB(int dir, int speed);
void init_motors(void);
int read_distance_sensors(void);
uint8_t read_sensor(int sensor);
uint16_t read_adc(int port);
void tc_init(void);
void mainLoop(void);
void signalLeds();

int main (void)
{
	board_init();
	sysclk_init();
	pmic_init();
	adc_init();
	init_motors();

	PORTE.DIRSET = 0xFF;
	PORTR.DIRSET = 0xFF;
	ioport_set_pin_dir(USER_LED0, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(USER_LED1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(MY_BUTTON, IOPORT_DIR_INPUT);
	ioport_get_pin_level(MY_BUTTON);
	ioport_get_pin_level(MY_BUTTON);
	
	while(1)
	{
		bool  state = ioport_get_pin_level(MY_BUTTON);
		if (state == 0)
		{
			break;
		}
		signalLeds();
	}
	
	ioport_set_pin_level(USER_LED0, 1);
	_delay_ms(1000);
	ioport_set_pin_level(USER_LED0, 0);
	_delay_ms(1000);
	ioport_set_pin_level(USER_LED0, 1);
	_delay_ms(1000);
	ioport_set_pin_level(USER_LED0, 0);
	_delay_ms(1000);
	ioport_set_pin_level(USER_LED0, 1);
	_delay_ms(1000);
	ioport_set_pin_level(USER_LED0, 0);
	
	tc_init();
	
	while (1)
	{
	}
}
int read_distance_sensors(void)
{
	int readings_L = read_adc(0) > DETECTVALUE;
	int readings_R = read_adc(1) > DETECTVALUE;
	if (readings_L == 1 || readings_R == 1)
	{
		_delay_ms(50);
		if (readings_L == 1)
		{
			readings_L = read_adc(0) > DETECTVALUE;
		}
		if (readings_R == 1)
		{
			readings_R = read_adc(1) > DETECTVALUE;
		}
	}
	int low_range = 0;
	for (int i = 0; i < 10; i++)
	{
		low_range += read_adc(2) < DETECTVALUE;
	}
	if (low_range == 10) low_range = 1;
	else low_range = 0;
	/*
	for (int i = 0; i < 2; ++i) {
		readings_L += read_adc(0) > DETECTVALUE;
		readings_R += read_adc(1) > DETECTVALUE;
		//_delay_ms(40);
	}
	if (readings_L == 2) readings_L = 1;
	else readings_L = 0;
	if (readings_R == 2) readings_R = 1;
	else readings_R = 0;
	*/
	
	if (readings_L)
	{
		ioport_set_pin_level(USER_LED0, 1);
	}
	else
	{
		ioport_set_pin_level(USER_LED0, 0);
	}
	if (readings_R)
	{
		ioport_set_pin_level(USER_LED1, 1);
	}
	else
	{
		ioport_set_pin_level(USER_LED1, 0);
	}
	if (low_range == 1)
	{
		ioport_set_pin_level(USER_LED0, 1);
		ioport_set_pin_level(USER_LED1, 1);
	}
	
	if (low_range == 1)
	{
		lastOponentPosition = 0;
		return 0;
	}
	else if (readings_L == 1 && readings_R == 1)
	{
		lastOponentPosition = 0;
		return 0;
	} 
	else if(readings_L == 0 && readings_R == 1)
	{
		lastOponentPosition = 1;
		return 1;
	}
	else if(readings_L == 1 && readings_R == 0)
	{
		lastOponentPosition = -1;
		return -1;
	}
	else if(readings_L == 0 && readings_R == 0)
	{
		if (lastOponentPosition == -1)
		{
			return -2;
		} 
		else
		{
			return 2;
		}
	}
	return -2;
}

#define SEEKMODE 0
#define CHASEMODE 1

int Mode = SEEKMODE;

void mainLoop()
{
	
	int oponentPosition = read_distance_sensors();
	if (Mode == SEEKMODE)
	{
		set_motors(MAXSPEED/1.5,50);
		if (oponentPosition |= -2)
		{
			Mode = CHASEMODE;
		}
	}
	else
	{
		if (oponentPosition == -2)
		{
			set_motors(MAXSPEED/4.5, -300);
		}
		else if (oponentPosition == -1)
		{
			set_motors(MAXSPEED/2.5, -200);
		}
		else if (oponentPosition == 0)
		{
			set_motors(MAXSPEED, 0);
		}
		else if (oponentPosition == 1)
		{
			set_motors(MAXSPEED/2.5, 200);
		}
		else if (oponentPosition == 2)
		{
			set_motors(MAXSPEED/4.5, 300);
		}
	}
}

void signalLeds()
{
	int readings_L = read_adc(0) > DETECTVALUE;
	int readings_R = read_adc(1) > DETECTVALUE;
	if (readings_L == 1 || readings_R == 1)
	{
		_delay_ms(50);
		if (readings_L == 1)
		{
			readings_L = read_adc(0) > DETECTVALUE;
		}
		if (readings_R == 1)
		{
			readings_R = read_adc(1) > DETECTVALUE;
		}
	}
	/*
	for (int i = 0; i < 1; ++i) {
		readings_L += read_adc(0) > DETECTVALUE;
		readings_R += read_adc(1) > DETECTVALUE;
		_delay_ms(38);
	}
	if (readings_L == 1) readings_L = 1;
	else readings_L = 0;
	if (readings_R == 1) readings_R = 1;
	else readings_R = 0;
	*/
	if (readings_L)
	{
		ioport_set_pin_level(USER_LED0, 1);
	}
	else
	{
		ioport_set_pin_level(USER_LED0, 0);
	}
	if (readings_R)
	{
		ioport_set_pin_level(USER_LED1, 1);
	}
	else
	{
		ioport_set_pin_level(USER_LED1, 0);
	}
}
void tc_init()
{
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0, mainLoop);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);
	tc_write_period(&TCC0, 32000); // 1kHz main loop
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
		if (turn <= 100)
		{
			speed_A = speed - speed * turn / 100;
			speed_B = speed;
			dir_A = 1;
			dir_B = 1;
		}
		else if(100 < turn && turn <= 200)
		{
			turn -= 100;
			speed_A = speed * turn / 100;
			speed_B = speed;
			dir_A = 0;
			dir_B = 1;
		}
		else if (200 < turn)
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
		if (-100 <= turn)
		{
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
	if (dir == -1)
	{
		PORTR.OUT |= PIN0_bm;
		PORTR.OUT &= ~PIN1_bm;
	}
	else if(dir == 1)
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