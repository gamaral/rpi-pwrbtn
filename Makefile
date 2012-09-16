BASE    := ..

TARGET  := rpi-pwrbtn
SRCS    := $(TARGET).c $(BASE)/common/main.c
OBJS    := $(SRCS:.c=.o)

CC      := avr-gcc
OBJCOPY := avr-objcopy
AVRDUDE := avrdude

#CFLAGS := -Os -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000UL -I$(BASE) # Arduino
CFLAGS  := -Os -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=8000000UL  -I$(BASE) # Generic & Arduino
LDFLAGS := -Os -Wl,--gc-sections -mmcu=atmega328p -lm
ADFLAGS := -p m328p -c avrisp2 -P usb -v

all: $(TARGET).hex
	@true

$(TARGET): $(TARGET).hex
	@true

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

flash: $(TARGET).hex
	@echo Flashing $(TARGET).hex
#	$(AVRDUDE) $(ADFLAGS) -e -U lock:w:0x3F:m -U lfuse:w:0xFF:m -U hfuse:w:0xD9:m -U efuse:w:0x05:m # Arduino
	$(AVRDUDE) $(ADFLAGS) -e -U lock:w:0x3F:m -U lfuse:w:0xE2:m -U hfuse:w:0xD9:m -U efuse:w:0x07:m # Generic & Arduino
	$(AVRDUDE) $(ADFLAGS) -U flash:w:$<

clean:
	-$(RM) $(OBJS) $(TARGET).elf $(TARGET).hex

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%: %.c
	$(MAKE) TARGET=$@ 

