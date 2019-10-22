# Location of build tools
CC=avr-gcc
CXX=avr-g++
OBJCOPY=avr-objcopy
SIZE=avr-size
AVRDUDE=avrdude

# Modify this to the device name of the UART used for AVRDUDE
AVRDUDE_DEV=usb
PROG = dragon_isp

# Modify this to the CPU you are using
PART=atmega328p
AVRDUDE_PART=m328p

# Cpu frequency is 16MHz, divider = 8
F_CPU="(16000000UL/8)"

# Directory for built objects
BUILD_DIR=build

# Port/application object files
APP_NAME = sw

# Application object files
APP_CXX_OBJECTS = main.o
APP_OBJECTS =

# Library object files to build and use
LIB_OBJECTS = uart.o
LIB_CXX_OBJECTS =
LIB_ASM_OBJECTS =
LIB_DIR = lib

# Collection of built objects
ALL_OBJECTS = $(LIB_OBJECTS) $(LIB_ASM_OBJECTS) $(APP_OBJECTS) $(APP_CXX_OBJECTS)
BUILT_OBJECTS = $(patsubst %,$(BUILD_DIR)/%,$(ALL_OBJECTS))

# Target application filenames (.elf and .hex) for each application object
APP_ELF = $(APP_NAME).elf
APP_HEX = $(APP_NAME).hex

# Search build/output directory for dependencies
vpath %.o ./$(BUILD_DIR)
vpath %.elf ./$(BUILD_DIR)
vpath %.hex ./$(BUILD_DIR)

# GCC flags
DEFINES=-DUART_RX0_BUFFER_SIZE=64 -DUART_TX0_BUFFER_SIZE=64
FLAGS=-g -mmcu=$(PART) -Os -Wall -Werror -DF_CPU=$(F_CPU) $(DEFINES)
CFLAGS=$(FLAGS) -std=c99
CXXFLAGS=$(FLAGS) -std=c++11
INCLUDES=-I. -I$(LIB_DIR)


#################
# Build targets #
#################

# All applications
all: $(BUILD_DIR) $(APP_HEX) Makefile

# Make build/output directory
$(BUILD_DIR):
	mkdir $(BUILD_DIR)

# Application HEX files
$(APP_HEX): %.hex: %.elf
	@echo Building $@
	$(OBJCOPY) -j .text -j .data -O ihex $(BUILD_DIR)/$< $(BUILD_DIR)/$@

# Application ELF files
$(APP_ELF): %.elf: $(LIB_OBJECTS) $(LIB_CXX_OBJECTS) $(LIB_ASM_OBJECTS) $(APP_OBJECTS) $(APP_CXX_OBJECTS)
	$(CXX) $(CXXFLAGS) $(BUILT_OBJECTS) --output $(BUILD_DIR)/$@ -Wl,-Map,$(BUILD_DIR)/$(basename $@).map

# Application objects builder
$(APP_OBJECTS): %.o: %.c
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $(BUILD_DIR)/$(notdir $@)

$(APP_CXX_OBJECTS): %.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDES) $< -o $(BUILD_DIR)/$(notdir $@)

# Application objects builder
$(LIB_OBJECTS): %.o: $(LIB_DIR)/%.c
	$(CC) -c $(CFLAGS) $(INCLUDES) $< -o $(BUILD_DIR)/$(notdir $@)

$(LIB_CXX_OBJECTS): %.o: $(LIB_DIR)/%.cpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDES) $< -o $(BUILD_DIR)/$(notdir $@)

$(LIB_ASM_OBJECTS): %.o: $(LIB_DIR)/%.s
	$(CC) -c $(CFLAGS) -x assembler-with-cpp $(INCLUDES) $< -o $(BUILD_DIR)/$(notdir $@)

# .lst file builder
%.lst: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -Wa,-al $< > $@

%.lst: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -Wa,-al $< > $@

# Clean
clean:
	rm -f *.o *.elf *.map *.hex *.bin *.lst
	rm -rf doxygen-avr
	rm -rf build

# Send to device
program: $(APP_HEX)
	$(SIZE) -C $(BUILD_DIR)/$(APP_ELF)
	$(AVRDUDE) $(AVRDUDE_FLAGS) -c $(PROG) -P $(AVRDUDE_DEV) -p $(AVRDUDE_PART) -U flash:w:$(BUILD_DIR)/$(APP_HEX) -v

fuse:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -c $(PROG) -P $(AVRDUDE_DEV) -p $(AVRDUDE_PART) -U lfuse:w:0x60:m -U hfuse:w:0xdc:m -U efuse:w:0xff:m -v

interactive:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -c $(PROG) -P $(AVRDUDE_DEV) -p $(AVRDUDE_PART) -t -v

doxygen:
	doxygen ./Doxyfile
