TARGET=rc5xmit
PORT=/dev/ttyACM0
AVRDUDE_DEVICE=m328p
PROGRAMMER=arduino

MCU=atmega328p
CC=avr-gcc
CXX=avr-g++
F_CPU=16000000
CFLAGS=-c -I./arduino -mmcu=$(MCU) -DF_CPU=$(F_CPU)
OUTPUTDIR=build
OTARGETS=$(OUTPUTDIR)/$(TARGET).cpp.o\
$(OUTPUTDIR)/hooks.c.o \
$(OUTPUTDIR)/wiring.c.o \
$(OUTPUTDIR)/wiring_digital.c.o \
$(OUTPUTDIR)/main.cpp.o \


all: $(OUTPUTDIR)/$(TARGET).hex

$(OUTPUTDIR)/$(TARGET).obj: $(OUTPUTDIR)/ $(OTARGETS)
	$(CC) $(OTARGETS) -o $@

$(OUTPUTDIR)/$(TARGET).cpp.o: $(TARGET).cpp
	$(CXX) $(CFLAGS) $< -o $@
$(OUTPUTDIR)/%.c.o: arduino/%.c
	$(CC) $(CFLAGS) $< -o $@
$(OUTPUTDIR)/%.cpp.o: arduino/%.cpp
	$(CXX) $(CFLAGS) $< -o $@

$(OUTPUTDIR)/$(TARGET).hex: $(OUTPUTDIR)/$(TARGET).obj
	avr-objcopy -R .eeprom -O ihex $< $@
program: $(OUTPUTDIR)/$(TARGET).hex
	avrdude -p $(AVRDUDE_DEVICE) -c $(PROGRAMMER) -P $(PORT) -U flash:w:$<

$(OUTPUTDIR)/:
	mkdir $(OUTPUTDIR)
clean:
	rm -rf $(OUTPUTDIR)/
