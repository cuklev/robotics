#define F_CPU 32000000UL

#include <asf.h>
#include "util/delay.h"

#define USER_LED(id)    IOPORT_CREATE_PIN(PORTE, id)

int main(void)
{
	int i;
	uint16_t leds=5;
	board_init();
	sysclk_init();

	PORTE.DIRSET = 0x00;
	for(i=0;i<8;++i)
		ioport_set_pin_dir(USER_LED(i), IOPORT_DIR_OUTPUT);
	
	while(1)
	{
		//ioport_set_pin_level(USER_LED0, 1);
		//ioport_set_pin_level(USER_LED1, 0);
		//_delay_ms(100);
		//ioport_set_pin_level(USER_LED0, 0);
		//ioport_set_pin_level(USER_LED1, 1);

		for(i=0;i<8;++i)
			ioport_set_pin_level(USER_LED(i), leds>>i&1);
		leds<<=1;
		if(leds>0xFF)leds=leds&0xFF|1;
		_delay_ms(100);
	}

	while (1) {}
}
