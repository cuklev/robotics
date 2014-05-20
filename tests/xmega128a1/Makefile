TARGET=main
PORT=usb
DEVICE=x128a1
MCU=atxmega128a1
PROGRAMMER=dragon_jtag

CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-DBOARD=USER_BOARD -DIOPORT_XMEGA_COMPAT -Os -fdata-sections -ffunction-sections -fdata-sections -Wall -c -mmcu=$(MCU) -std=gnu99 -fno-strict-aliasing -Wstrict-prototypes -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -mrelax $(INCLUDE_DIRS)
ASMFLAGS=-Wa,-gdwarf2 -x assembler-with-cpp -DBOARD=USER_BOARD -c -mmcu=$(MCU) -mrelax $(INCLUDE_DIRS)
LDFLAGS=-Wl,--start-group -Wl,--end-group -Wl,--gc-sections -Wl,--relax -mmcu=$(MCU)

all: $(TARGET).hex
clean:
	rm -f $(TARGET).o $(TARGET).obj $(TARGET).hex
program: $(TARGET).hex
	avrdude -p $(DEVICE) -c $(PROGRAMMER) -P $(PORT) -U flash:w:$<
$(TARGET).hex: $(TARGET).obj
	$(OBJCOPY) -R .eeprom -O ihex $< $@
$(TARGET).obj: $(TARGET).o
	$(CC) $(LDFLAGS) $< -o $@
$(TARGET).o: $(TARGET).c
	$(CC) $(CFLAGS) $< -o $@
