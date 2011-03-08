###############################################################################
# Makefile for the project Serial_Communication_ZigBee_wireless
###############################################################################

## General Flags
PROJECT = Serial_Communication_ZigBee_wireless
MCU = atmega2560
TARGET = Serial_Communication_ZigBee_wireless.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99     -DF_CPU=14745600UL -O0 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=Serial_Communication_ZigBee_wireless.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = Serial_Communication_ZigBee_wireless.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) Serial_Communication_ZigBee_wireless.hex Serial_Communication_ZigBee_wireless.eep Serial_Communication_ZigBee_wireless.lss size

## Compile
Serial_Communication_ZigBee_wireless.o: ../Serial_Communication_ZigBee_wireless.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) Serial_Communication_ZigBee_wireless.elf dep/* Serial_Communication_ZigBee_wireless.hex Serial_Communication_ZigBee_wireless.eep Serial_Communication_ZigBee_wireless.lss Serial_Communication_ZigBee_wireless.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

