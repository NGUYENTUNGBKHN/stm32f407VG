# Makefile for the bootloader project
TARGET_NAME:=test
BUILD_DIR:=_build
MAKE_COMMON:=../Common

# This is the root project directory and the core directory
ROOT_PROJ:=./
# Core directory for the project
CORE_DIR:=$(ROOT_PROJ)/../Core
CMSIS_DIR:=$(ROOT_PROJ)/../Core/cmsis/inc
EXTERNAL_DIR:=$(ROOT_PROJ)/../External
MIDDLER_DIR:=$(ROOT_PROJ)/../Middleware
WORKSPACE_DIR:=$(ROOT_PROJ)/../
DRIVER_DIR:=$(ROOT_PROJ)/../Driver
HAL_DIR:=$(ROOT_PROJ)/../Driver/Hal
OS_DIR:=$(ROOT_PROJ)/../OS/ThreadX

# This is the out file rule
$(BUILD_DIR)/$(TARGET_NAME).out: \
	LINKER_SCRIPT := $(ROOT_PROJ)/$(TARGET_NAME).ld

# This is the Flags for the compiler
CPU=-mcpu=cortex-m4 -mthumb -mabi=aapcs
FPU=-mfloat-abi=hard -mfpu=fpv4-sp-d16
OPT=-O3 -g3
CCFLAGS+=$(CPU) $(FPU) $(OPT)
CCFLAGS+=-ffunction-sections -fdata-sections -fno-strict-aliasing
CCFLAGS+=-fno-builtin -fshort-enums
CCFLAGS+=-Wall -Wno-unused-variable

ASMFLAG+=$(CPU) $(FPU) $(OPT)

LDFLAGS+=$(CPU) $(FPU) $(OPT)
LDFLAGS+=-Wl,--gc-sections
LDFLAGS+=--specs=nano.specs
LDFLAGS+=-T$(LINKER_SCRIPT) -mthumb -mabi=aapcs
LDFLAGS+=-nostartfiles -ereset_handler


# SRC_FILES=\
# 	$(ROOT_PROJ)/main.c\
# 	$(ROOT_PROJ)/boot_startup.c\

SRC_EXCLUDE=\

SRC_INCLUDE+=$(wildcard *.c)
SRC_INCLUDE+=$(wildcard *.S)

# SRC_FILES+=$(wildcard $(OSKERNEL_DIR)/*.s)
SRC_FILES:=$(filter-out $(SRC_EXCLUDE), $(SRC_INCLUDE))

INC_FOLDERS=\
	$(OS_DIR)/common/inc\
	$(OS_DIR)/ports/cortex-m4/inc\

LIBS+=$(OS_DIR)/_build/tx.a
# LIBS+=-lc -lm


.PHONY:	default help

default: $(TARGET_NAME)

# Include common makefile. File contains TOOLCHAIN directory and rules for building targets
include $(MAKE_COMMON)/Makefile.common

# Call function to define targets 
$(foreach target, $(TARGET_NAME), $(call define_target, $(target)))


# Help target
help:
	@echo  help

# Flash target
flash: default
	Jlink.exe -if swd -device nRF52840_xxAA -speed 4000 -CommanderScript CommanderScript/build_windows.jlink

# Erase target
erase:
	Jlink.exe -if swd -device nRF52840_xxAA -speed 4000 -CommanderScript CommanderScript/erase_windows.jlink


