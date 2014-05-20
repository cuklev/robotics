TARGET=blink
PORT=/dev/ttyACM0
AVRDUDE_DEVICE=m328p
PROGRAMMER=arduino

MCU=atmega328p
CC=avr-gcc
CXX=avr-g++
F_CPU=16000000
CFLAGS=-c -I./arduino -mmcu=$(MCU) -O0 -pipe -fno-strict-aliasing -DF_CPU=$(F_CPU)
OTARGETS=build/$(TARGET).cpp.o \
build/hooks.c.o \
build/wiring.c.o \
build/wiring_digital.c.o \
build/main.cpp.o

all: build/$(TARGET).obj

build/$(TARGET).obj: build/ $(OTARGETS)
	$(CC) $(OTARGETS) -o $@

build/$(TARGET).cpp.o: $(TARGET).cpp
	$(CXX) $(CFLAGS) $< -o $@
build/%.c.o: arduino/%.c
	$(CC) $(CFLAGS) $< -o $@
build/%.cpp.o: arduino/%.cpp
	$(CXX) $(CFLAGS) $< -o $@

build/$(TARGET).hex: build/$(TARGET).obj
	avr-objcopy -R .eeprom -O ihex $< $@
program: build/$(TARGET).hex
	avrdude -p $(AVRDUDE_DEVICE) -c $(PROGRAMMER) -P $(PORT) -U flash:w:$<

build/:
	mkdir build
clean:
	rm -rf build/
