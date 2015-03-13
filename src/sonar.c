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

#include "sonar.h"
#include "time.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

// TODO: Adjust this value based on the temperature
#define US_ROUNDTRIP_CM 57 // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space. Default=57

void SonarHandler(void) {
    static uint32_t prev = 0;
    static bool last_edge = false;
    //static uint32_t prev_micros = 0;

    TimerIntClear(WTIMER0_BASE, TIMER_CAPA_EVENT); // Clear interrupt
    uint32_t curr = TimerValueGet(WTIMER0_BASE, TIMER_A); // Read capture value
    bool edge = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4); // Read the GPIO pin

    if (last_edge && !edge) { // Check that we are going from a positive to falling egde
        uint32_t diff = curr - prev; // Calculate diff
        uint32_t diff_us = 1000000UL / (SysCtlClockGet() / diff); // Convert to us
#if 1
        //UARTprintf("%u %u %u %d\n", diff, diff_us, diff_us / US_ROUNDTRIP_CM,  micros() - prev_micros);
        UARTprintf("%u %u\n", diff_us, diff_us / US_ROUNDTRIP_CM);
#else
        // TODO: Check if pulse is within range
        // TODO: Store in global buffer
        // TODO: Take average of several measurements
#endif
    }

    prev = curr; // Store previous value
    last_edge = edge; // Store last edge
    //prev_micros = micros();
}

void triggerHandler(void) {
    static bool pinValue = false; // It is low to begin with

    TimerIntClear(WTIMER0_BASE, TIMER_TIMB_TIMEOUT); // Clear interrupt

    if (!pinValue) { // If we were just low, send out a 5us trigger pulse
        TimerLoadSet(WTIMER0_BASE, TIMER_B, SysCtlClockGet() / 200000 - 1); // Reset timeout value to 5us
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, GPIO_PIN_5); // Set pin high
    } else {
        TimerLoadSet(WTIMER0_BASE, TIMER_B, SysCtlClockGet() / 40 - 1); // Set to interrupt in 25ms
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // Set pin low
    }

    pinValue = !pinValue;
}

// WTimer0A is used to measure the width of the sonar ehco pulse
// WTimer0B is used to send out an approximately 5us trigger pulse
void initSonar(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0); // Enable Wide Timer0 peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable GPIOC peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinConfigure(GPIO_PC4_WT0CCP0); // Use alternate function
    GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_4); // Use pin with timer peripheral

    // Split timers and enable timer A event up-count timer and timer B as a periodic timer
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP | TIMER_CFG_B_PERIODIC);

    // Configure WTimer0A
    TimerControlEvent(WTIMER0_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES); // Interrupt on both edges
    TimerIntRegister(WTIMER0_BASE, TIMER_A, SonarHandler); // Register interrupt handler
    TimerIntEnable(WTIMER0_BASE, TIMER_CAPA_EVENT); // Enable timer capture A event interrupt
    IntPrioritySet(INT_WTIMER0A, 0); // Configure Wide Timer 0A interrupt priority as 0
    IntEnable(INT_WTIMER0A); // Enable Wide Timer 0A interrupt

    // Configure WTimer0B
    TimerLoadSet(WTIMER0_BASE, TIMER_B, SysCtlClockGet() / 40 - 1); // Set to interrupt in 25ms
    TimerIntRegister(WTIMER0_BASE, TIMER_B, triggerHandler); // Register interrupt handler
    TimerIntEnable(WTIMER0_BASE, TIMER_TIMB_TIMEOUT); // Enable timer timeout interrupt
    IntPrioritySet(INT_WTIMER0B, 0); // Configure Wide Timer 0B interrupt priority as 0
    IntEnable(INT_WTIMER0B); // Enable Wide Timer 0B interrupt

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5); // Set PC5 as output
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0); // Set pin low

    TimerEnable(WTIMER0_BASE, TIMER_BOTH); // Enable both timers
}
