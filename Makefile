TC32_BIN = tc32_toolchain/linux/bin
CC = $(TC32_BIN)/tc32-elf-gcc
LD = $(TC32_BIN)/tc32-elf-ld
OBJCOPY = $(TC32_BIN)/tc32-elf-objcopy
OBJDUMP = $(TC32_BIN)/tc32-elf-objdump
SIZE = $(TC32_BIN)/tc32-elf-size

SDK_PLATFORM = tc_platform_sdk
CHIP = B85
SDK_CHIP = $(SDK_PLATFORM)/chip/$(CHIP)

INCLUDES = -I. \
           -I$(SDK_CHIP)/drivers \
           -I$(SDK_CHIP)/drivers/lib/include \
           -I$(SDK_PLATFORM)/common

CFLAGS = -fno-builtin-printf -fno-builtin-puts -ffunction-sections -fdata-sections -Wall -O2 \
         -fpack-struct -fshort-enums -finline-small-functions \
         -std=gnu99 -fshort-wchar -fms-extensions \
         -DMCU_STARTUP_FLASH -DMCU_CORE_B85=1 $(INCLUDES)

LDFLAGS = -T $(SDK_PLATFORM)/project/tlsr_tc32/$(CHIP)/boot.link \
          --gc-sections

LIBS = $(SDK_CHIP)/drivers/lib/libdriver.a

DRIVER_OBJS = bsp.o analog.o clock.o gpio.o uart.o timer.o flash.o watchdog.o string.o

OBJS = cstartup_flash.o div_mod.o $(DRIVER_OBJS) printf_uart.o main.o

all: hanshow_packet_sniffer.bin

cstartup_flash.o: $(SDK_CHIP)/boot/cstartup_flash.S
	$(CC) $(CFLAGS) -c -o $@ $<

div_mod.o: $(SDK_PLATFORM)/common/div_mod.S
	$(CC) $(CFLAGS) -c -o $@ $<

bsp.o: $(SDK_CHIP)/drivers/bsp.c
	$(CC) $(CFLAGS) -c -o $@ $<

analog.o: $(SDK_CHIP)/drivers/analog.c
	$(CC) $(CFLAGS) -c -o $@ $<

clock.o: $(SDK_CHIP)/drivers/clock.c
	$(CC) $(CFLAGS) -c -o $@ $<

gpio.o: $(SDK_CHIP)/drivers/gpio.c
	$(CC) $(CFLAGS) -c -o $@ $<

uart.o: $(SDK_CHIP)/drivers/uart.c
	$(CC) $(CFLAGS) -c -o $@ $<

timer.o: $(SDK_CHIP)/drivers/timer.c
	$(CC) $(CFLAGS) -c -o $@ $<

flash.o: $(SDK_CHIP)/drivers/flash.c
	$(CC) $(CFLAGS) -c -o $@ $<

watchdog.o: $(SDK_CHIP)/drivers/watchdog.c
	$(CC) $(CFLAGS) -c -o $@ $<

string.o: $(SDK_PLATFORM)/common/string.c
	$(CC) $(CFLAGS) -c -o $@ $<

printf_uart.o: printf_uart.c printf_uart.h
	$(CC) $(CFLAGS) -c -o $@ $<

main.o: main.c printf_uart.h
	$(CC) $(CFLAGS) -c -o $@ $<

hanshow_packet_sniffer.elf: $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

hanshow_packet_sniffer.bin: hanshow_packet_sniffer.elf
	$(OBJCOPY) -O binary $< $@
	$(SIZE) $<

clean:
	rm -f *.o *.elf *.bin *.lst
