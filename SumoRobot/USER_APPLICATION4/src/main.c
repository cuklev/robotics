#define F_CPU 32000000UL

#include <asf.h>
#include "util/delay.h"

#define USER_LED0    IOPORT_CREATE_PIN(PORTD, 5)
#define USER_LED1    IOPORT_CREATE_PIN(PORTD, 4)
#define MY_BUTTON    IOPORT_CREATE_PIN(PORTD, 0)

#define MY_ADC    ADCA
#define MY_ADC_CH ADC_CH0

#define DETECTVALUE 3000
#define MAXSPEED 80
#define TURNSPEED 40
#define BLINDSPEED 40

struct pwm_config pwm_cfg[2];
int lastdir=-300;
int autostop=5000;
bool sen_L_Front;
bool sen_R_Front;

uint16_t read_adc(int port);
void adc_init(void);
void init_motors(void);
void set_motorA(int dir, int speed);
void set_motorB(int dir, int speed);
void set_motors(int speed, int turn);
void signalLeds(void);
void mainLoop(void);

int main(void)
{
	// Hardware initialisation
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
	
	// Wait for button
	while(ioport_get_pin_level(MY_BUTTON))
	{
		signalLeds();
		_delay_ms(10);
	}

//	FANCY _delay_ms(5000);
	for(int i=0;i<5000;i+=50)
	{
		if(i%500==0)ioport_set_pin_level(USER_LED0, 0);
		else if(i%500==400)ioport_set_pin_level(USER_LED0, 1);
		if(i%150==0)ioport_set_pin_level(USER_LED1, 0);
		else if(i%150==50)ioport_set_pin_level(USER_LED1, 1);
		_delay_ms(50);
	}

	// Timer counter start
	tc_enable(&TCC0);
	tc_set_overflow_interrupt_callback(&TCC0, mainLoop);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);
	tc_write_period(&TCC0, 32000); // 1kHz main loop
	tc_set_overflow_interrupt_level(&TCC0, TC_INT_LVL_LO);
	cpu_irq_enable();
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV1_gc);
	
	// MainLoop
	set_motors(MAXSPEED/2, -300);
	while (1) {}
}

uint16_t read_adc(int port)
{
	struct adc_channel_config adcch_conf;
	
	adcch_read_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
	
	adcch_set_input(&adcch_conf, port, ADCCH_NEG_INTERNAL_GND, 1);
	
	adcch_write_configuration(&MY_ADC, MY_ADC_CH, &adcch_conf);
	
	adc_enable(&MY_ADC);

	adc_start_conversion(&MY_ADC, MY_ADC_CH);
	adc_wait_for_interrupt_flag(&MY_ADC, MY_ADC_CH);

	return adc_get_result(&MY_ADC, MY_ADC_CH);
}

void adc_init(void)
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

void init_motors(void)
{
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

void set_motors(int speed, int turn)
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

void signalLeds(void)
{
	sen_L_Front = read_adc(4) < DETECTVALUE;
	sen_R_Front = read_adc(5) < DETECTVALUE;

	ioport_set_pin_level(USER_LED0, sen_L_Front);
	ioport_set_pin_level(USER_LED1, sen_R_Front);
}

void mainLoop(void)
{
	signalLeds();

	if(!autostop)
	{
		--autostop;
		set_motors(0, 0);
		return;
	}

	if (sen_L_Front && sen_R_Front)
	{
		set_motors(MAXSPEED, 0);
	}
	else if (sen_L_Front && !sen_R_Front)
	{
		lastdir=300;
		set_motors(TURNSPEED, 200);
	}
	else if (!sen_L_Front && sen_R_Front)
	{
		lastdir=-300;
		set_motors(TURNSPEED, -200);
	}
	else set_motors(BLINDSPEED, lastdir);
}
