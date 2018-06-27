# Make file for compiling a kilobot program, both for the
# kilobot simulator and for the real kilobot.
# The program can consist of multiple files. Files and paths to various libraries are
# configured below, these needs to be adapted to the system.

#list of c files in this project
SOURCES=morphogenesis.c util.c

#name of the executable file
EXECUTABLE=morphogenesis

# Path to the official kilolib, used for real bots only
# Set it here, or as an environment variable in the shell:
#      export KILOLIB=/your/path/  
#
KILOHEADERS=$(KILOLIB_PATH)

# path of userdata.h distributed with the simulator but also needed
# when compiling the user program for the real kilobot (avr-gcc has different default paths)
SIMHEADERS=/usr/local/include

#path to kilolib.a in the official kilolib, needed for real bots only
KILOLIB    =$(KILOHEADERS)/build/kilolib.a

#compilation flags for simulated version
SIM_CFLAGS = -c -g -O2 -Wall -std=c99  #-I$(KILOHEADERS)

#linking flags for simulated version
SIM_LFLAGS = -lsim -lSDL -lm -ljansson

# linking flags to compile headless
# SIM_LFLAGS = -lheadless  -lm -ljansson


# Makefile targets.
# sim (default target) is the simulator program
# hex is the .hex file (program) for the real kilobot.
# all builds both.

# Note: the hex targer requires the AVR toolchain and
# the kilolib library to be installed. See simulator README.

sim: $(EXECUTABLE)
hex: $(EXECUTABLE).hex
all: sim hex

clean :
	rm *.o $(EXECUTABLE) *.elf *.hex

# # # # # # # # # # The following should be generic and not need changes # # # # # # # # # # # # # 

# compiling for the real bot
#

CC = avr-gcc
AVRAR = avr-ar
AVROC = avr-objcopy
AVROD = avr-objdump
AVRUP = avrdude

#PFLAGS = -P usb -c avrispmkII # user to reprogram OHC
CFLAGS = -mmcu=atmega328p -Wall -gdwarf-2 -O3 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -DF_CPU=8000000 -I$(KILOHEADERS) -I$(SIMHEADERS) -DKILOBOT

#for a floating point printf
CFLAGS += -Wl,-u,vfprintf -lprintf_flt -lm

ASFLAGS = $(CFLAGS)

FLASH = -R .eeprom -R .fuse -R .lock -R .signature
EEPROM = -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0  

%.lss: %.elf
	$(AVROD) -d -S $< > $@

%.hex: %.elf
	$(AVROC) -O ihex $(FLASH) $< $@

%.eep: %.elf
	$(AVROC) -O ihex $(EEPROM) $< $@

%.bin: %.elf
	$(AVROC) -O binary $(FLASH) $< $@ 

$(EXECUTABLE).elf: $(SOURCES) $(KILOLIB)
	$(CC) $(CFLAGS) -o $@ $^ 

# make syntax:
# $@ left-hand side of :
# $^ right-hand side of :
# $< first item of right-hand side of :



# From here on, we compile for the simulator
# 

#which c compiler to use. gcc works well.
SIM_CC=gcc

#automatically make a list of object files from the list of .c files
OBJECTS=$(SOURCES:.c=.o)


#general rule for compiling a .c file to .o
.c.o:
	$(SIM_CC) $(SIM_CFLAGS) $< -o $@

$(EXECUTABLE): $(OBJECTS) $(SIMLIB) 
	$(SIM_CC)  $(SIM_LFLAGS) -o $@  $(OBJECTS) 


