DEVICES := DiWheelDrive PowerManagement LightRing

DiWheelDrive := 1
PowerManagement := 2
LightRing := 3

SERIALBOOT ?= $(dir $(abspath $(lastword $(MAKEFILE_LIST))))../../amiro-blt/Host/Source/SerialBoot/build/SerialBoot

ifeq ($(OS),Windows_NT)
	SERIALBOOT_PORT ?= COM4
else
	SERIALBOOT_PORT ?= /dev/ttyUSB0
#	SERIALBOOT_PORT ?= /dev/rfcomm0
endif
SERIALBOOT_BAUDRATE ?= 115200
#SERIALBOOT_BAUDRATE ?= 921600

#SERIALBOOT_BT_ADDR ?= 00:00:00:00:00:00
SERIALBOOT_BT_ADDR ?= 00:07:80:44:23:F9

ifdef PROJECT
	DEVICES :=
	FLASHFILE = build/$(PROJECT).srec
	FLASHTRGS = $(FLASHFILE)
	FLASHARGS =  -T$($(PROJECT)) $(FLASHFILE)
	FLASHARGSDIRECT = $(FLASHFILE)
else
	FLASHTRGS = $(foreach dev,$(DEVICES),$(dev)/build/$(dev).srec)
	FLASHARGS = $(foreach dev,$(DEVICES),-T$($(dev)) $(dev)/build/$(dev).srec)
	FLASHARGSDIRECT = $(foreach dev,$(DEVICES),$(dev)/build/$(dev).srec)
endif

flash: $(DEVICES) $(FLASHTRGS)
	@echo Flashing
	$(SERIALBOOT) -d$(SERIALBOOT_PORT) -b$(SERIALBOOT_BAUDRATE) $(FLASHARGS)

flashdirect: $(DEVICES) $(FLASHTRGS)
	@echo Flashing
	$(SERIALBOOT) -d$(SERIALBOOT_PORT) -b$(SERIALBOOT_BAUDRATE) $(FLASHARGSDIRECT)

flashbt: $(DEVICES) $(FLASHTRGS)
	@echo Flashing over Bluetooth to $(SERIALBOOT_BT_ADDR)
	$(SERIALBOOT) -a$(SERIALBOOT_BT_ADDR) $(FLASHARGS)

