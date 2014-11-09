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

#include "RX.h"
#include "UART.h"
#include "time.h"
#include "ppm.h"

#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

#if 0
#define GPIO_RX_PERIPH		SYSCTL_PERIPH_GPIOB // Test code for switch 1
#define GPIO_RX_BASE      GPIO_PORTB_BASE
#define GPIO_RX_PIN       GPIO_PIN_5
#else
#define GPIO_RX_PERIPH		SYSCTL_PERIPH_GPIOF
#define GPIO_RX_BASE      GPIO_PORTF_BASE
#define GPIO_RX_PIN       GPIO_PIN_4
#endif

void RxHandler(void) {
	GPIOIntClear(GPIO_RX_BASE, GPIO_RX_PIN); // Clear interrupt source
	//UARTprintf("Millis: %d\n", millis());
}

void initIO(void) {
	SysCtlPeripheralEnable(GPIO_RX_PERIPH); // Enable GPIO peripheral

	GPIOPinTypeGPIOInput(GPIO_RX_BASE, GPIO_RX_PIN); // Set as input
#if GPIO_RX_BASE == GPIO_PORTF_BASE && (GPIO_RX_PIN == GPIO_PIN_0 || GPIO_RX_PIN == GPIO_PIN_4) // Check if using built-in switch
	GPIO_PORTF_PUR_R |= GPIO_RX_PIN; // Enable pull-up
#endif

	GPIOIntTypeSet(GPIO_RX_BASE, GPIO_RX_PIN, GPIO_BOTH_EDGES); // Enable interrupt on both falling and rising edge
	GPIOIntRegister(GPIO_RX_BASE, RxHandler); // Register interrupt handler
	GPIOIntEnable(GPIO_RX_BASE, GPIO_RX_PIN); // Enable interrupt
}

int main(void) {
	// Set the clocking to run directly from the external crystal/oscillator.
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set clock to 80MHz (400MHz(PLL) / 2 / 2.5 = 80 MHz)

	initUART();
	initTime();
	initRX();
	initIO();
	initPPM();

	IntMasterEnable();
	
	delay(100); // Wait a little for everything to initialize

	UARTprintf("CLK %d\n", SysCtlClockGet());
#if 0
	const uint16_t min = getPeriod() / (20000/1060); // From SimonK firmware
	const uint16_t max = getPeriod() / (20000/1860); // From SimonK firmware
#else
	const uint16_t min = 1060; // From SimonK firmware
	const uint16_t max = 1860; // From SimonK firmware
#endif
	uint16_t width = min;

	UARTprintf("min: %d, max: %d, period: %d\n", min, max, getPeriod());

	while (1) {
		UARTprintf("Width: %d\n", width);
		//writePPMWidth(width++);
		writePPMUs(width++);
		if (width > max)
			width = min;
		delay(2);
	}
}

// TODO:
	// Compare CPPM values with measured values from oscilloscope
	// SPI/MPU-6050
	// PPM output signal
		// Sync all PPM output signals
	// EEPROM
