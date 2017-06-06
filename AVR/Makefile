#----------------------------------------------------------------------------
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make upload = Download the hex file to the device, using avrdude.
#               Please customize the avrdude settings below first!
#
# make filename.o = Just compile filename.c into the object code only.
#
#----------------------------------------------------------------------------
#=========================================================================
#     ENVIRONMENT AND PATHING
#=========================================================================
OBJDIR:=.
BASEDIR:=$(CURDIR)
SRCDIR:=$(BASEDIR)

# Tells Make where to find the files you are including
vpath %.c $(SRCDIR)
vpath %.h $(SRCDIR)

# If these are defined in your environment it will override them
# Put the c files you wish to include in this file
MAKE_SOURCES?=$(BASEDIR)/sources.mk

include $(MAKE_SOURCES)

#=========================================================================
#     PROGRAMS
#=========================================================================
CC:=avr-gcc
LD:=avr-gcc
OBJCOPY:=avr-objcopy
OBJDUMP:=avr-objdump
SIZE:=avr-size
INSTALL:=avrdude
NM = avr-nm
WINSHELL = cmd
#=========================================================================
#     CONFIGURATION
#=========================================================================
MCU:=atmega328p
F_CPU:=16000000

#Target mappings
TARGET:=main
TARGET_ELF:=$(TARGET).elf
TARGET_HEX:=$(TARGET).hex
TARGET_MAP:=$(TARGET).map

# Install file format
FORMAT:=ihex

#Installer defines
INSTALL_PROG:=arduino
INSTALL_PORT:=COM3
INSTALL_FLAGS:=-F -V -c $(INSTALL_PROG) -p $(MCU) -P $(INSTALL_PORT)
INSTALL_FLASH:=-U flash:w:$(TARGET_HEX)

#Run time defines
CDEFS:=-DF_CPU=$(F_CPU)UL

#Linker flags
LDFLAGS:=-Wl,-Map,$(TARGET_ELF:%.elf=%.map),--cref -mmcu=$(MCU)

# Define all object files.
OBJS:=$(SRCS:%.c=$(OBJDIR)/%.o)
DEP_FILES:=$(OBJS:%.o=%.d)
CFLAGS:=-mmcu=$(MCU) -Os $(CDEFS) -Wall

#=========================================================================
#     MAKEFILE TARGET DEFINIIONS
#=========================================================================
# Display size of file.
HEXSIZE:=$(SIZE) --target=$(FORMAT) $(TARGET_HEX)
ELFSIZE:=$(SIZE) --mcu=$(MCU) --format=avr $(TARGET_ELF)

# Display compiler version information.
gccversion:
	@$(CC) --version

.PHONY: all 
all: $(TARGET_HEX)
.PHONY: build
build: $(OBJS)
.PHONY: link
link: $(TARGET_ELF) 
.PHONY: convert
convert: $(TARGET_HEX) 
.PHONY: gen_asm
gen_asm: $(TARGET_ASM)
.PHONY: clean
.PHONY: upload
 
$(TARGET_ELF): $(OBJS)
	$(LD) $(LDFLAGS) -o $(TARGET_ELF) $(OBJS)
	$(ELFSIZE)

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O $(FORMAT) $(TARGET_ELF) $(TARGET_HEX)
	$(HEXSIZE)

upload: $(TARGET_HEX)
	$(INSTALL) $(INSTALL_FLAGS) $(INSTALL_FLASH)

# Target: clean project.
clean:
	rm $(TARGET_ELF) $(TARGET_HEX) $(OBJS) $(TARGET_MAP)
