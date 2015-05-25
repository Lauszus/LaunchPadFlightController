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
PART=TM4C123GH6PM

#
# The base directory for TivaWare.
#
ROOT=../lib

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find source files that do not live in this directory.
#
VPATH=${ROOT}/utils

#
# Where to find header files that do not live in the source directory.
#
IPATH=${ROOT}

#
# Macros passed to the preprocessor.
#
MACROS=-DUART_BUFFERED -DUART_BUFFERED1 -DUART_DEBUG=1 -DUSE_MAG=1 -DUSE_BARO=0

#
# The default rule, which causes the project to be built.
#
all: ${COMPILER} ${COMPILER}/LaunchPadFlightController.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to clean out all the build products.
#
flash: all
	lm4flash ${COMPILER}/LaunchPadFlightController.bin

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

#
# Rules for building the project.
#
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/main.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/BMP180.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/Bluetooth.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/Buzzer.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/EEPROM.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/HMC5883L.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/I2C.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/IMU.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/MPU6500.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/PID.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/PPM.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/RX.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/Sonar.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/Time.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/UART.o
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/uartstdio1.o

${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/uartstdio.o
${COMPILER}/LaunchPadFlightController.axf: ${ROOT}/driverlib/${COMPILER}/libdriver.a
${COMPILER}/LaunchPadFlightController.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/LaunchPadFlightController.axf: LaunchPadFlightController.ld
SCATTERgcc_LaunchPadFlightController=LaunchPadFlightController.ld
ENTRY_LaunchPadFlightController=ResetISR
CFLAGSgcc=-DTARGET_IS_BLIZZARD_RB1

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
