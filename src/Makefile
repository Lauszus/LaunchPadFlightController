#******************************************************************************
#
# Makefile - Rules for building the interrupts example.
#
# Copyright (c) 2012-2013 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
#
# Texas Instruments (TI) is supplying this software for use solely and
# exclusively on TI's microcontroller products. The software is owned by
# TI and/or its suppliers, and is protected under applicable copyright
# laws. You may not combine this software with "viral" open-source
# software in order to form a larger program.
#
# THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
# NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
# NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
# CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES, FOR ANY REASON WHATSOEVER.
#
# This is part of revision 1.1 of the EK-TM4C123GXL Firmware Package.
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
PART ?= TM4C123GH6PM

#
# Set target name.
#
TARGET ?= LaunchPadFlightController

#
# Preprocessor directives.
#
UART_DEBUG ?= 0
USE_MAG ?= 1
USE_BARO ?= 1
USE_SONAR ?= 1
USE_LIDAR_LITE ?= 1
USE_FLOW_SENSOR ?= 1
ONESHOT125 ?= 1
STEP_ACRO_SELF_LEVEL ?= 0
STEP_ALTITUDE_HOLD ?= 0
STEP_HEADING_HOLD ?= 0

BUILD_FLAGS = -DUART_DEBUG=$(UART_DEBUG) -DUSE_MAG=$(USE_MAG) -DUSE_BARO=$(USE_BARO) -DUSE_SONAR=$(USE_SONAR) -DUSE_LIDAR_LITE=$(USE_LIDAR_LITE) -DUSE_FLOW_SENSOR=$(USE_FLOW_SENSOR) -DONESHOT125=$(ONESHOT125)

#
# The base directory for TivaWare.
#
ROOT = ../lib

#
# Include the common make definitions.
#
include $(ROOT)/makedefs

#
# Where to find header files that do not live in the source directory.
#
IPATH = $(ROOT)

#
# Path to project object file.
#
PROJECT_OBJ = $(COMPILER)/$(TARGET).axf

#
# The default rule, which causes the project to be built.
#
all: $(COMPILER) $(PROJECT_OBJ)
	@echo "\n  Build flags: $(BUILD_FLAGS)\n"
	@$(PREFIX)-size $(PROJECT_OBJ)

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf $(COMPILER) $(wildcard *~)

#
# The rule to build and flash the object file.
#
flash: all
	@lm4flash -v $(COMPILER)/$(TARGET).bin

#
# The rule to create the target directory.
#
$(COMPILER):
	@mkdir -p $(COMPILER)

#
# Create a list of all object files.
#
TARGET_OBJS = $(addsuffix .o,$(addprefix $(COMPILER)/,$(basename $(notdir $(wildcard *.c)))))

#
# Rules for building the project.
#
$(PROJECT_OBJ): $(TARGET_OBJS)
$(PROJECT_OBJ): $(ROOT)/utils/uartstdio.o
$(PROJECT_OBJ): $(ROOT)/driverlib/$(COMPILER)/libdriver.a
$(PROJECT_OBJ): $(TARGET).ld

SCATTERgcc_$(TARGET) = $(TARGET).ld
ENTRY_$(TARGET) = ResetISR

#
# CFLAGS passed to the compiler.
#
CFLAGSgcc =  -DTARGET_IS_BLIZZARD_RB1 -DUART_BUFFERED -DUART_BUFFERED1
CFLAGSgcc += $(BUILD_FLAGS)
CFLAGSgcc += -DSTEP_ACRO_SELF_LEVEL=$(STEP_ACRO_SELF_LEVEL) -DSTEP_ALTITUDE_HOLD=$(STEP_ALTITUDE_HOLD) -DSTEP_HEADING_HOLD=$(STEP_HEADING_HOLD)
CFLAGSgcc += -Wdouble-promotion -Wunsafe-loop-optimizations -Wunused-parameter

#
# Make all warnings into errors when building using Travis CI
#
ifdef TRAVIS_CI
CFLAGSgcc += -Werror
endif

#
# Rebuild everything when makefile changes.
#
$(TARGET_OBJS): Makefile

#
# Include the automatically generated dependency files.
#
ifneq ($(MAKECMDGOALS),clean)
-include $(wildcard $(COMPILER)/*.d) __dummy__
endif
