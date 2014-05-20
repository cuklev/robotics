TARGET=main
PORT=usb
AVRDUDE_DEVICE=x32a4u
PROGRAMMER=dragon_pdi

INCLUDE_DIRS=\
-I. \
-I./config \
-I./ASF \
-I./ASF/xmega \
-I./ASF/xmega/utils \
-I./ASF/xmega/utils/bit_handling \
-I./ASF/xmega/utils/assembler \
-I./ASF/xmega/utils/preprocessor \
-I./ASF/xmega/drivers \
-I./ASF/xmega/drivers/usart \
-I./ASF/xmega/drivers/cpu \
-I./ASF/xmega/drivers/tc \
-I./ASF/xmega/drivers/rtc \
-I./ASF/xmega/drivers/sleep \
-I./ASF/xmega/drivers/pmic \
-I./ASF/xmega/drivers/adc \
-I./ASF/xmega/drivers/adc/xmega_aau \
-I./ASF/xmega/drivers/nvm \
-I./ASF/xmega/services \
-I./ASF/xmega/services/pwm \
-I./ASF/common \
-I./ASF/common/utils \
-I./ASF/common/utils/interrupt \
-I./ASF/common/utils/stdio \
-I./ASF/common/utils/stdio/stdio_serial \
-I./ASF/common/boards \
-I./ASF/common/boards/user_board \
-I./ASF/common/services \
-I./ASF/common/services/sleepmgr \
-I./ASF/common/services/sleepmgr/xmega \
-I./ASF/common/services/serial \
-I./ASF/common/services/serial/xmega_usart \
-I./ASF/common/services/clock \
-I./ASF/common/services/clock/xmega \
-I./ASF/common/services/ioport \
-I./ASF/common/services/ioport/xmega

OTARGETS=\
out/main.o \
out/init.o \
out/sysclk.o \
out/sleepmgr.o \
out/adc.o \
out/adc_aau.o \
out/ccp.o \
out/nvm_asm.o \
out/tc.o \
out/pwm.o

PREFIX=/home/krisi/Downloads/avr/avr8-gnu-toolchain-linux_x86_64
CC=$(PREFIX)/bin/avr-gcc-4.8.1
OBJCOPY=$(PREFIX)/bin/avr-objcopy
MCU=atxmega32a4u
CFLAGS=-DBOARD=USER_BOARD -DIOPORT_XMEGA_COMPAT -Os -fdata-sections -ffunction-sections -fdata-sections -Wall -c -mmcu=atxmega32a4u -std=gnu99 -fno-strict-aliasing -Wstrict-prototypes -Wmissing-prototypes -Werror-implicit-function-declaration -Wpointer-arith -mrelax $(INCLUDE_DIRS)
ASMFLAGS=-Wa,-gdwarf2 -x assembler-with-cpp -DBOARD=USER_BOARD -c -mmcu=atxmega32a4u -mrelax $(INCLUDE_DIRS)
LDFLAGS=-Wl,--start-group -Wl,--end-group -Wl,--gc-sections -Wl,--relax -mmcu=atxmega32a4u

all: out/$(TARGET).obj
clean:
	rm -rf out/
out:
	mkdir out

out/$(TARGET).hex: out/$(TARGET).obj
	$(OBJCOPY) -R .eeprom -O ihex $< $@
program: out/$(TARGET).hex
	avrdude -p $(AVRDUDE_DEVICE) -c $(PROGRAMMER) -P $(PORT) -U flash:w:$<

out/$(TARGET).obj: out $(OTARGETS)
	$(CC) $(LDFLAGS) $(OTARGETS) -o $@


# C files

out/usart.o: ./ASF/xmega/drivers/usart/usart.c
	$(CC) $(CFLAGS) $< -o $@
out/tc.o: ./ASF/xmega/drivers/tc/tc.c
	$(CC) $(CFLAGS) $< -o $@
out/rtc.o: ./ASF/xmega/drivers/rtc/rtc.c
	$(CC) $(CFLAGS) $< -o $@
out/adc.o: ./ASF/xmega/drivers/adc/adc.c
	$(CC) $(CFLAGS) $< -o $@
out/adc_aau.o: ./ASF/xmega/drivers/adc/xmega_aau/adc_aau.c
	$(CC) $(CFLAGS) $< -o $@
out/nvm.o: ./ASF/xmega/drivers/nvm/nvm.c
	$(CC) $(CFLAGS) $< -o $@
out/pwm.o: ./ASF/xmega/services/pwm/pwm.c
	$(CC) $(CFLAGS) $< -o $@
out/read.o: ./ASF/common/utils/stdio/read.c
	$(CC) $(CFLAGS) $< -o $@
out/write.o: ./ASF/common/utils/stdio/write.c
	$(CC) $(CFLAGS) $< -o $@
out/init.o: ./ASF/common/boards/user_board/init.c
	$(CC) $(CFLAGS) $< -o $@
out/sleepmgr.o: ./ASF/common/services/sleepmgr/xmega/sleepmgr.c
	$(CC) $(CFLAGS) $< -o $@
out/usart_serial.o: ./ASF/common/services/serial/usart_serial.c
	$(CC) $(CFLAGS) $< -o $@
out/sysclk.o: ./ASF/common/services/clock/xmega/sysclk.c
	$(CC) $(CFLAGS) $< -o $@
out/ioport_compat.o: ./ASF/common/services/ioport/xmega/ioport_compat.c
	$(CC) $(CFLAGS) $< -o $@
out/main.o: ./main.c
	$(CC) $(CFLAGS) $< -o $@

out/ccp.o: ./ASF/xmega/drivers/cpu/ccp.s
	$(CC) $(ASMFLAGS) $< -o $@
out/nvm_asm.o: ./ASF/xmega/drivers/nvm/nvm_asm.s
	$(CC) $(ASMFLAGS) $< -o $@
