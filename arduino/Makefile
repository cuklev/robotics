TARGET=blink
PORT=/dev/ttyACM0
AVRDUDE_DEVICE=m328p

CC=avr-gcc
CXX=avr-g++
CFLAGS=-c -I. -mmcu=atmega328p -Os
OTARGETS=$(TARGET).cpp.o wiring_digital.c.o wiring.c.o hooks.c.o

all: $(TARGET).obj

$(TARGET).hex: $(TARGET).obj
	avr-objcopy -R .eeprom -O ihex $< $@

$(TARGET).obj: $(OTARGETS)
	$(CC) $(LDFLAGS) $(OTARGETS) -o $@

%.c.o: %.c
	$(CC) $(CFLAGS) $< -o $@

%.cpp.o: %.cpp
	$(CXX) $(CFLAGS) $< -o $@

program: $(TARGET).hex
	avrdude -p $(AVRDUDE_DEVICE) -c arduino -P $(PORT) -U flash:w:$(TARGET).hex

clean:
	rm -rf *.o *.obj *.hex
