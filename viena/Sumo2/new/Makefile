TARGET=main
PORT=usb
DEVICE=x32a4u
MCU=atxmega32a4u
PROGRAMMER=dragon_pdi

PREFIX=~/Downloads/avr/avr8-gnu-toolchain-linux_x86_64/bin
CC=$(PREFIX)/avr-g++
OBJCOPY=$(PREFIX)/avr-objcopy
CFLAGS=-DBOARD=USER_BOARD -DIOPORT_XMEGA_COMPAT -Os -fdata-sections -ffunction-sections -Wall -c -mmcu=$(MCU) -fno-strict-aliasing -Werror-implicit-function-declaration -Wpointer-arith -mrelax
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
$(TARGET).o: $(TARGET).cpp Makefile
	$(CC) $(CFLAGS) $< -o $@
