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

#if UART_DEBUG

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "EEPROM.h"
#include "Time.h"
#include "UART.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#include "utils/ustdlib.h"

void initUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable the GPIO port containing the pins that will be used.
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    // Configure the GPIO pin muxing for the UART function.
    // This is only necessary if your part supports GPIO pin function muxing.
    // Study the data sheet to see which functions are allocated per pin.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Since GPIO A0 and A1 are used for the UART function, they must be
    // configured for use as a peripheral function (instead of GPIO).
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig(0, 115200, SysCtlClockGet()); // Mode is set to 8N1 on UART0
    UARTEchoSet(false);

    while (UARTBusy(UART0_BASE)) {
        // Wait until UART is ready
    }

    UARTprintf("Started\n");
}

void printPIDValues(pid_t *pid) {
    UARTprintf("PID: %d.%04u\t%d.%04u\t%d.%04u\t%d.%04u\n",
                                            (int16_t)pid->Kp, (uint16_t)(abs(pid->Kp * 10000.0f) % 10000),
                                            (int16_t)pid->Ki, (uint16_t)(abs(pid->Ki * 10000.0f) % 10000),
                                            (int16_t)pid->Kd, (uint16_t)(abs(pid->Kd * 10000.0f) % 10000),
                                            (int16_t)pid->integrationLimit, (uint16_t)(abs(pid->integrationLimit * 10000.0f) % 10000));
    UARTFlushTx(false);
}

static void setValues(char *input) {
    if (input[0] == 'G' && input[1] == 'P') { // Send "GP;" to get the current PID Values
        printPIDValues(&cfg.pidRoll); // Print PID Values
        printPIDValues(&cfg.pidYaw);
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
        UARTprintf("%s\n", dataInput); // Echo message back
        setValues(dataInput);
    }
}

#else

#include "PID.h"

void initUART(void) {
}
void checkUARTData(void) {
}
void printPIDValues(pid_t *pid) {
}

#endif
