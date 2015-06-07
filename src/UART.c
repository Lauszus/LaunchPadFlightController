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

#if UART_DEBUG

#include "EEPROM.h"
#include "Time.h"
#include "UART.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

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

void printPIDValues(pid_values_t *pidValues) {
    UARTprintf("PID: %d.%04u\t%d.%04u\t%d.%05u\t%d.%04u\n",
                                            (int16_t)pidValues->Kp, (uint16_t)(abs(pidValues->Kp * 10000.0f) % 10000),
                                            (int16_t)pidValues->Ki, (uint16_t)(abs(pidValues->Ki * 10000.0f) % 10000),
                                            (int16_t)pidValues->Kd, (uint16_t)(abs(pidValues->Kd * 100000.0f) % 100000),
                                            (int16_t)pidValues->integrationLimit, (uint16_t)(abs(pidValues->integrationLimit * 10000.0f) % 10000));
    UARTFlushTx(false);
}

void printSettings(void) {
    UARTprintf("Settings: %d.%02u\t%d.%02u\t%u\t%u\t%d.%02u\t%d.%02u\n",
                                            (int16_t)cfg.angleKp, (uint16_t)(abs(cfg.angleKp * 100.0f) % 100),
                                            (int16_t)cfg.headKp, (uint16_t)(abs(cfg.headKp * 100.0f) % 100),
                                            cfg.maxAngleInclination, cfg.maxAngleInclinationSonar,
                                            (int16_t)cfg.stickScalingRollPitch, (uint16_t)(abs(cfg.stickScalingRollPitch * 100.0f) % 100),
                                            (int16_t)cfg.stickScalingYaw, (uint16_t)(abs(cfg.stickScalingYaw * 100.0f) % 100));
    UARTFlushTx(false);
}

#endif
