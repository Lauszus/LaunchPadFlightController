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

#include "Sonar.h"
#include "Time.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#if UART_DEBUG
//#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

// Implemented based on: http://che126.che.caltech.edu/28015-PING-Sensor-Product-Guide-v2.0.pdf

// TODO: Adjust this value based on the temperature
#define US_ROUNDTRIP_CM 57 // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space. Default=57

static volatile int16_t sonarDistanceUs;

static void SonarHandler(void) {
    static uint32_t prev = 0;
    static bool last_edge = false;

    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT); // Clear interrupt
    uint32_t curr = TimerValueGet(WTIMER0_BASE, TIMER_A); // Read capture value
    bool edge = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4); // Read the GPIO pin

    if (last_edge && !edge) { // Check that we are going from a positive to falling edge
        uint32_t diff = curr - prev; // Calculate diff
        sonarDistanceUs = 1000000UL / (SysCtlClockGet() / diff); // Convert to us
        //UARTprintf("%u %u\n", sonarDistanceUs, sonarDistanceUs / US_ROUNDTRIP_CM);
        // TODO: Take average of several measurements using DMA
    }

    prev = curr; // Store previous value
    last_edge = edge; // Store last edge
}

void triggerSonar(void) {
    static uint32_t lastTrigger = 0;

    uint32_t now = millis();
    if ((int32_t)(now - lastTrigger) > 25) { // Trigger every 25ms
        lastTrigger = now;
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); // Set pin high
        delayMicroseconds(10); // Other sources wait 10us
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // Set pin low
        //UARTprintf("%d\n", getSonarDistance());
    }
}

// Returns the distance in cm. Range is 0-300 cm.
int16_t getSonarDistance(void) {
    int16_t distance = sonarDistanceUs / US_ROUNDTRIP_CM;
    if (distance > 300) // Datasheet says 3m is maximum
        distance = -1;
    return distance;
}

// WTimer0A is used to measure the width of the sonar echo pulse
void initSonar(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0); // Enable Wide Timer0 peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable GPIOC peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinConfigure(GPIO_PC4_WT0CCP0); // Use alternate function
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4); // Use pin with timer peripheral

    // Split timers and enable timer A event up-count timer
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);

    // Configure WTimer0A
    TimerControlEvent(WTIMER0_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES); // Interrupt on both edges
    TimerIntRegister(WTIMER0_BASE, TIMER_A, SonarHandler); // Register interrupt handler
    TimerIntEnable(WTIMER0_BASE, TIMER_CAPA_EVENT); // Enable timer capture A event interrupt
    IntPrioritySet(INT_WTIMER0A, 0); // Configure Wide Timer 0A interrupt priority as 0
    IntEnable(INT_WTIMER0A); // Enable Wide Timer 0A interrupt

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5); // Set PC5 as output
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // Set pin low

    TimerEnable(WTIMER0_BASE, TIMER_A); // Enable Wide Timers 0A
}
