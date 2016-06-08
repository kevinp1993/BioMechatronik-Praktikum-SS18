##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -fomit-frame-pointer -falign-functions=16 -DCHPRINTF_USE_FLOAT=1 -fstack-usage -DSERIAL_BUFFERS_SIZE=64
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -fno-exceptions -std=c++11 -U__STRICT_ANSI__
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Enables the use of FPU on Cortex-M4.
# Possible selections are:
#   no     - no FPU is used (propably equals 'soft')
#   soft   - does not use the FPU, thus all floating point operations are emulated
#   softfp - uses the FPU, but uses the integer registers only
#   hard   - uses the FPU and passes data via the FPU registers
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# Enable this if you really want to use the STM FWLib.
ifeq ($(USE_FWLIB),)
  USE_FWLIB = no
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = PowerManagement

# Imported source files and paths
CHIBIOS = ../../../ChibiOS
AMIRO = ../..
include $(AMIRO)/boards/PowerManagement/board.mk
include $(CHIBIOS)/os/hal/platforms/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/ports/GCC/ARMCMx/STM32F4xx/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk
include $(CHIBIOS)/os/various/cpp_wrappers/kernel.mk

# Define linker script file here
LDSCRIPT= $(BOARDLD)/STM32F405xG.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(CHIBIOS)/os/various/evtimer.c \
       $(CHIBIOS)/os/various/syscalls.c \
       $(CHIBIOS)/os/various/chprintf.c \
       $(CHIBIOS)/os/various/shell.c \
       $(CHIBIOS)/os/various/memstreams.c \
       $(AMIRO)/stubs.c \
       $(AMIRO)/components/Debug.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(CHCPPSRC) \
         $(AMIRO)/components/Color.cpp \
         $(AMIRO)/components/ControllerAreaNetworkRx.cpp \
         $(AMIRO)/components/ControllerAreaNetworkTx.cpp \
         $(AMIRO)/components/bus/i2c/HWI2CDriver.cpp \
         $(AMIRO)/components/bus/i2c/I2CMultiplexer.cpp \
         $(AMIRO)/components/bus/i2c/VI2CDriver.cpp \
         $(AMIRO)/components/proximity/vcnl4020.cpp \
         $(AMIRO)/components/eeprom/eeprom.cpp \
         $(AMIRO)/components/eeprom/at24.cpp \
         $(AMIRO)/components/FileSystemInputOutput/FileSystemInputOutputBase.cpp \
         $(AMIRO)/components/FileSystemInputOutput/FSIOPowerManagement.cpp \
         $(AMIRO)/components/input/mpr121.cpp \
         $(AMIRO)/components/power/adconverter.cpp \
         $(AMIRO)/components/power/ina219.cpp \
         $(AMIRO)/components/power/bq27500.cpp \
         $(AMIRO)/components/bluetooth/bluetooth-descriptor.cpp \
         $(AMIRO)/components/bluetooth/bluetooth-transport.cpp \
         $(AMIRO)/components/bluetooth/bluetooth-connector.cpp \
         $(AMIRO)/components/bluetooth/bluetooth-wiimote.cpp \
         $(AMIRO)/components/bluetooth/bluetooth-serial.cpp \
         $(AMIRO)/components/bluetooth/bluetooth-iwrap.cpp \
         $(AMIRO)/components/bluetooth/bluetooth.cpp \
         $(AMIRO)/components/serial_reset/iwrap_can_mux.cpp \
         $(AMIRO)/components/serial_reset/serial_can_mux.cpp \
         PowerManagement.cpp \
         userthread.cpp \
         main.cpp \
         exti.cpp

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(PORTASM)

INCDIR = $(PORTINC) $(KERNINC) $(TESTINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) \
         $(CHCPPINC) \
         $(CHIBIOS)/os/various \
         $(AMIRO) \
         $(AMIRO)/include

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
#LD   = $(TRGT)gcc
LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary
SREC = $(CP) -O srec --srec-len=248

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -D_DEBUG=1
DDEFS =

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS =

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user defines
##############################################################################

ifeq ($(USE_FWLIB),yes)
  include $(CHIBIOS)/ext/stm32lib/stm32lib.mk
  CSRC += $(STM32SRC)
  INCDIR += $(STM32INC)
  USE_OPT += -DUSE_STDPERIPH_DRIVER
endif

RULESPATH = $(CHIBIOS)/os/ports/GCC/ARMCMx
include $(RULESPATH)/rules.mk

include $(AMIRO)/ports/rules.mk
include $(AMIRO)/devices/flash.mk
