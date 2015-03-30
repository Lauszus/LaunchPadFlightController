/* Copyright (C) 2015 Kristian Lauszus, TKJ Electronics. All rights reserved.

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

#include "Bluetooth.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "uartstdio1.h" // Add "UART_BUFFERED1" to preprocessor - it uses a modified version of uartstdio, so it can be used with another UART interface

void initBluetooth(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable the GPIO port containing the pins that will be used.
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    // Configure the GPIO pin muxing for the UART function.
    // This is only necessary if your part supports GPIO pin function muxing.
    // Study the data sheet to see which functions are allocated per pin.
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);

    // Since GPIO B0 and B1 are used for the UART function, they must be
    // configured for use as a peripheral function (instead of GPIO).
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig1(1, 115200, SysCtlClockGet()); // Mode is set to 8N1 on UART1
    UARTEchoSet1(false);

    while(UARTBusy(UART1_BASE)) {
        // Wait until UART is ready
    }

    UARTprintf1("Started Bluetooth\n");
}

static char dataInput[100]; // Use this buffer to store the incoming values

void readBluetooth(void) {
    if (UARTRxBytesAvail1()) {
        uint8_t i = 0;
        while (1) {
            dataInput[i] = UARTgetc1(); // This is a blocking call
            if (dataInput[i] == ';') // Keep reading until it reads a semicolon
                break;
            if (++i >= sizeof(dataInput) / sizeof(dataInput[0]) - 1) // String is too long
                return;
        }
        dataInput[i + 1] = '\0'; // Add null-character
        UARTprintf1("%s\n", dataInput); // Echo message back
    }
}
