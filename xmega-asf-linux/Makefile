MCU = atxmega128a1
AVRDUDE_DEVICE = x128a1
MCU ?= atxmega128a1
AVRDUDE_DEVICE ?= x128a1

CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) -Os -std=c99
CC=avr-gcc
OBJ2HEX=avr-objcopy 
LDFLAGS=-Wl,-gc-sections -Wl,-relax

INCLUDES = \
       -Iasf-3.12.1/common/boards                                      \
       -Iasf-3.12.1/common/services/clock                              \
       -Iasf-3.12.1/common/services/gfx_mono                           \
       -Iasf-3.12.1/common/services/gfx_mono/unit_tests                \
       -Iasf-3.12.1/common/services/gfx_mono/unit_tests/atxmega128a1_xplain \
       -Iasf-3.12.1/common/services/gpio                               \
       -Iasf-3.12.1/common/services/ioport                             \
       -Iasf-3.12.1/common/services/serial                             \
       -Iasf-3.12.1/common/services/serial/xmega_usart                 \
       -Iasf-3.12.1/common/utils                                       \
       -Iasf-3.12.1/common/utils/stdio/stdio_serial                    \
       -Iasf-3.12.1/xmega/boards                                       \
       -Iasf-3.12.1/xmega/boards/xplain                                \
       -Iasf-3.12.1/xmega/drivers/cpu                                  \
       -Iasf-3.12.1/xmega/drivers/pmic                                 \
       -Iasf-3.12.1/xmega/drivers/usart                                \
       -Iasf-3.12.1/xmega/utils                                        \
       -Iasf-3.12.1/xmega/utils/preprocessor \
       -Iasf-3.12.1/common/services/gfx_mono/unit_tests/atxmega128a1_xplain/gcc

CFLAGS+= $(INCLUDES)

PORT ?= usb
AVRDUDE=avrdude

TARGET=test
OBJECT_FILES=test.o

all: $(TARGET).hex

clean:
	rm -f *.o *.hex *.obj *.hex

%.hex: %.obj
	$(OBJ2HEX) -R .eeprom -O ihex $< $@

test.o: test.c

%.obj: $(OBJECT_FILES)
	echo $(INCLUDES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@
	echo $(INCLUDES)

wtf:
	echo $(INCLUDES)
	echo $(CFLAGS)

program: $(TARGET).hex
	$(AVRDUDE) -p $(AVRDUDE_DEVICE) -c dragon_jtag -P $(PORT) -U flash:w:$(TARGET).hex
