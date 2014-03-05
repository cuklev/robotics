#define F_CPU 32000000UL

#include <asf.h>
#include <avr/io.h>
#include <util/delay.h>

#define BLINK_DELAY_MS 100

int main()
{
	board_init();
	sysclk_init();

	PORTE.DIR = 0b11111111;
	
	uint8_t bitpattern = 0b00010001;
	while(1)
	{
		PORTE.OUT = bitpattern;
		_delay_ms( BLINK_DELAY_MS );
		bitpattern <<= 1;
		
		if (bitpattern == 0b00010000) bitpattern = 0b00010001;
	}
}
