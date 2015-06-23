CPP     = arm-none-eabi-cpp
CC      = arm-none-eabi-gcc
AS      = arm-none-eabi-as
AR      = arm-none-eabi-ar
LD      = arm-none-eabi-ld
CFLAGS  = -g3 -Os -ffreestanding -ffunction-sections -fdata-sections -std=c99 -Wall -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wstrict-prototypes -Wmissing-prototypes -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard $(DEFINES) $(INCLUDES)
ASFLAGS = -g3 -Os -mcpu=cortex-m4 -mthumb -mabi=aapcs  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -D__ASSEMBLY__ $(DEFINES) $(INCLUDES)
LDFLAGS = -g3 -Os -mcpu=cortex-m4 -mthumb -mabi=aapcs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -nostartfiles -Wl,--gc-sections,--no-undefined -T./external/Device/TI/TM4C123/Sources/GCC/link_TM4C123.ld --specs=nano.specs
INCLUDES = -I./external/CMSIS/Include -I./external/Device/TI/TM4C123/Include -I./external -I. 
DEFINES  = -DTM4C123GH6PM -D__STACK_SIZE=4096
SRCS = \
	armv7m_core.c \
	armv7m_exception.S \
	armv7m_pendsv.c \
	armv7m_profile.c \
	armv7m_safe.c \
	armv7m_stack.c \
	armv7m_systick.c \
	armv7m_udelay.c \
	tm4c123_button.c \
	tm4c123_capture.c \
	tm4c123_clib.c \
	tm4c123_disk.c \
	tm4c123_i2c.c \
	tm4c123_led.c \
	tm4c123_receiver.c \
	tm4c123_servo.c \
	tm4c123_spi.c \
	tm4c123_tft.c \
	tm4c123_uart.c \
	tm4c123_udma.c \
	ak8975.c \
	calibration.c \
	control.c \
	display.c \
	ekf_core.c \
	ekf_math.c \
	fifo.c \
	gps.c \
	guidance.c \
	hmc5883.c \
	mission.c \
	mpu6050.c \
	navigation.c \
	record.c \
	rfat_core.c \
	rfat_disk.c \
	tft.c \
	kitty.c

OBJS = \
	armv7m_core.o \
	armv7m_exception.o \
	armv7m_pendsv.o \
	armv7m_profile.o \
	armv7m_safe.o \
	armv7m_stack.o \
	armv7m_systick.o \
	armv7m_udelay.o \
	tm4c123_button.o \
	tm4c123_capture.o \
	tm4c123_clib.o \
	tm4c123_disk.o \
	tm4c123_i2c.o \
	tm4c123_led.o \
	tm4c123_receiver.o \
	tm4c123_servo.o \
	tm4c123_spi.o \
	tm4c123_tft.o \
	tm4c123_uart.o \
	tm4c123_udma.o \
	ak8975.o \
	calibration.o \
	control.o \
	display.o \
	ekf_core.o \
	ekf_math.o \
	fifo.o \
	gps.o \
	guidance.o \
	hmc5883.o \
	mission.o \
	mpu6050.o \
	navigation.o \
	record.o \
	rfat_core.o \
	rfat_disk.o \
	tft.o \
	kitty.o

all:: kitty.elf

startup_TM4C123.o:: ./external/Device/TI/TM4C123/Sources/GCC/startup_TM4C123.S
	$(CC) $(ASFLAGS) -c $< -o $@

system_TM4C123.o:: ./external/Device/TI/TM4C123/Sources/system_TM4C123.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(ASFLAGS) -c $< -o $@

kitty.elf:: startup_TM4C123.o system_TM4C123.o $(OBJS)
	$(CC) $(LDFLAGS) startup_TM4C123.o system_TM4C123.o $(OBJS) -lm -lc -lgcc -o kitty.elf

clean::
	rm -f kitty.elf *~ *.o *.d

