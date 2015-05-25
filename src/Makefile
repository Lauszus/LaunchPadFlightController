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
# The default rule, which causes the project to be built.
#
all: $(COMPILER) $(COMPILER)/$(TARGET).axf
	@echo
	$(PREFIX)-size $(COMPILER)/$(TARGET).axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf $(COMPILER) $(wildcard *~)

#
# The rule to clean out all the build products.
#
flash: all
	lm4flash -v $(COMPILER)/$(TARGET).bin

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
$(COMPILER)/$(TARGET).axf: $(TARGET_OBJS)
$(COMPILER)/$(TARGET).axf: $(ROOT)/utils/uartstdio.o
$(COMPILER)/$(TARGET).axf: $(ROOT)/driverlib/$(COMPILER)/libdriver.a
$(COMPILER)/$(TARGET).axf: $(TARGET).ld

SCATTERgcc_$(TARGET) = $(TARGET).ld
ENTRY_$(TARGET) = ResetISR

#
# CFLAGS passed to the compiler.
#
CFLAGSgcc = -DTARGET_IS_BLIZZARD_RB1 -DUART_BUFFERED -DUART_BUFFERED1 -DUART_DEBUG=1 -DUSE_MAG=1 -DUSE_BARO=0

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
