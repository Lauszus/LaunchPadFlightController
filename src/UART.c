/* Copyright (C) 2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "PID.h"
#include "time.h"
#include "UART.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#include <utils/ustdlib.h>

void initUART(void) {
	// Enable the GPIO port containing the pins that will be used.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Configure the GPIO pin muxing for the UART function.
	// This is only necessary if your part supports GPIO pin function muxing.
	// Study the data sheet to see which functions are allocated per pin.
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

	// Since GPIO A0 and A1 are used for the UART function, they must be
	// configured for use as a peripheral function (instead of GPIO).
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTStdioConfig(0, 115200, SysCtlClockGet()); // Mode is set to 8N1
	UARTEchoSet(false);
}

void printPIDValues(void) {
	UARTprintf("%d.%04d\t%d.%04d\t%d.%04d\n",
																	(int16_t)pidRoll.Kp, (int16_t)abs(pidRoll.Kp * 10000.0f) % 10000,
																	(int16_t)pidRoll.Ki, (int16_t)abs(pidRoll.Ki * 10000.0f) % 10000,
																	(int16_t)pidRoll.Kd, (int16_t)abs(pidRoll.Kd * 10000.0f) % 10000);
	UARTFlushTx(false);
}

void setValues(char *input) {
  if (input[0] == 'G' && input[1] == 'P') // Send "GP;" to get the current PID Values
		printPIDValues(); // Print PID Values
  else if (input[0] == 'S') { // Set different values
		if (input[1] == 'P') {
			char *pStart = ustrstr(input, "SP,") + 3; // Find location of "SP,"
			pidRoll.Kp = ustrtof(pStart, NULL);
		} else if (input[1] == 'I') {
			char *pStart = ustrstr(input, "SI,") + 3; // Find location of "SI,"
			pidRoll.Ki = ustrtof(pStart, NULL);
		} else if (input[1] == 'D') {
			char *pStart = ustrstr(input, "SD,") + 3; // Find location of "SD,"
			pidRoll.Kd = ustrtof(pStart, NULL);
		}
		pidPitch = pidRoll; // Use same PID values for both pitch and roll
		printPIDValues(); // Print new PID Values
	}
}

static char dataInput[100]; // Use this buffer to store the incoming values

void checkUARTData(void) {
	if (UARTRxBytesAvail()) {
		uint8_t i = 0;
		while (1) {
			dataInput[i] = UARTgetc(); // This is a blocking call
			if (dataInput[i] == ';') // Keep reading until it reads a semicolon
				break;
			if (++i >= sizeof(dataInput) / sizeof(dataInput[0]) - 1) // String is too long
				return;
		}
		dataInput[i + 1] = '\0'; // Add null-character
		UARTprintf("%s\n", dataInput); // Echo send chars back
		setValues(dataInput);
	}
}
