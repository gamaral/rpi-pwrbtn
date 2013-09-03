BASE    := ..

TARGET  := main
SRCS    := $(TARGET).c
OBJS    := $(SRCS:.c=.o)

CC      := avr-gcc
OBJCOPY := avr-objcopy
AVRDUDE := avrdude

#CLOCK  := 8000000UL
CLOCK   := 1000000UL
CFLAGS  := -Os -ffunction-sections -fdata-sections -mmcu=attiny85 -DF_CPU=$(CLOCK) -I$(BASE)
LDFLAGS := -Os -Wl,--gc-sections -mmcu=attiny85 -lm
ADFLAGS := -p t85 -c avrisp2 -P usb -v

all: $(TARGET).hex $(TARGET).eep.hex
	@true

$(TARGET): $(TARGET).hex $(TARGET).eep.hex
	@true

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

$(TARGET).eep.hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $< $@

flash: $(TARGET).hex $(TARGET).eep.hex
	@echo Flashing $(TARGET).hex
	$(AVRDUDE) $(ADFLAGS) -e -U lfuse:w:0x62:m -U hfuse:w:0xDF:m -U efuse:w:0xFF:m # 1 MHz
	$(AVRDUDE) $(ADFLAGS) -U flash:w:$(TARGET).hex -U eeprom:w:$(TARGET).eep.hex

clean:
	-$(RM) $(OBJS) $(TARGET).elf $(TARGET).hex $(TARGET).eep.hex

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%: %.c
	$(MAKE) TARGET=$@ 

