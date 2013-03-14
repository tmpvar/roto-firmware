
MCU                   = atmega32u4
ARCH                  = AVR8
BOARD                 = USBKEY
F_CPU                 = 16000000
F_USB                 = $(F_CPU)
FLASH_SIZE_KB         = 32
BOOT_SECTION_SIZE_KB  = 4
OPTIMIZATION          = s
TARGET                = roto
SRC                   = $(TARGET).c Descriptors.c $(LUFA_SRC_USB) $(LUFA_SRC_USBCLASS)
LUFA_PATH             = ../../LUFA
CC_FLAGS              = -DUSE_LUFA_CONFIG_HEADER -IConfig/
LD_FLAGS              =
AVRDUDE_PROGRAMMER    = avr109
AVRDUDE_PORT          ?= /dev/tty.usbmodemfd121
AVRDUDE_FLAGS         ?= -v -v -v -v -v -v -v -v -v -v -v -v

tpad:

tpad flash --filename=$(TARGET).hex --port=AVRDUDE_PORT

# Default target
all:

# Include LUFA build script makefiles
include $(LUFA_PATH)/Build/lufa_core.mk
include $(LUFA_PATH)/Build/lufa_sources.mk
include $(LUFA_PATH)/Build/lufa_build.mk
include $(LUFA_PATH)/Build/lufa_cppcheck.mk
include $(LUFA_PATH)/Build/lufa_doxygen.mk
include $(LUFA_PATH)/Build/lufa_dfu.mk
include $(LUFA_PATH)/Build/lufa_avrdude.mk
include $(LUFA_PATH)/Build/lufa_atprogram.mk

