#define F_CPU 32000000UL

#include<avr/interrupt.h>
#include<avr/eeprom.h>

void setClockTo32MHz();
void init();

volatile uint8_t bits=0;
uint8_t*addr=(uint8_t*)42;

int main()
{
	init();
	PORTE.OUT=~eeprom_read_byte(addr);
	while(1);
}

void init()
{
	PORTD.DIR=0x00; // бутончета
	PORTE.DIR=0xFF; // светодиоди

	PORTD.PIN0CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTD.PIN1CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTD.PIN2CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTD.PIN3CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTD.PIN4CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!
	PORTD.PIN5CTRL=PORT_OPC_PULLUP_gc | PORT_ISC_FALLING_gc; // BUTTONS!!!

	cli();
	PORTD.INT0MASK=0x3F;
	PORTD.INTCTRL=PMIC_LOLVLEN_bm;

	PMIC.CTRL|=7;
	sei();
}

ISR(PORTD_INT0_vect)
{
	bits|=~PORTD.IN&0x3F;
	eeprom_write_byte(addr,bits);
}
