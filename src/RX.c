/* Copyright (C) 2015 Kristian Sloth Lauszus. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Sloth Lauszus
 Web      :  http://www.lauszus.com
 e-mail   :  lauszus@gmail.com
*/

#include <stdint.h>
#include <stdbool.h>

#include "Buzzer.h"
#include "Config.h"
#include "PPM.h"
#include "RX.h"
#include "Time.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#if UART_DEBUG
//#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#define RX_MID_INPUT ((RX_MAX_INPUT + RX_MIN_INPUT) / 2)

volatile bool validRXData;

static volatile uint16_t rxChannel[RX_NUM_CHANNELS];
static uint32_t timerLoadValue;

static void CaptureHandler(void) {
    static uint8_t channelIndex = 0;
    static uint32_t prev = 0;

    TimerIntClear(RX_TIMER_BASE, RX_INT_FLAG); // Clear interrupt
    uint32_t curr = TimerValueGet(RX_TIMER_BASE, RX_TIMER); // Read capture value

    uint32_t diff = curr - prev; // Calculate diff
    prev = curr; // Store previous value
    uint32_t diff_us = 1000000UL / (SysCtlClockGet() / diff); // Convert to us
    if (diff_us > 2700) { // Check if sync pulse is received - see: https://github.com/multiwii/baseflight/blob/master/src/drv_pwm.c
        channelIndex = 0; // Reset channel index
        validRXData = true;
        for (uint8_t i = 0; i < RX_NUM_CHANNELS; i++) {
            if (rxChannel[i] < 500 || rxChannel[i] > 2500) // Make sure that all are within a valid range
                validRXData = false;
        }
        if (validRXData)
            TimerLoadSet(RX_LOST_TIMER_BASE, RX_LOST_TIMER, timerLoadValue); // Reset timeout value to 100ms
#if 0 && UART_DEBUG
        for (uint8_t i = 0; i < RX_NUM_CHANNELS; i++) {
            if (rxChannel[i] > 0)
                UARTprintf("%u\t", rxChannel[i]);
            else
                break;
        }
        UARTprintf("\n");
#endif
    } else if (channelIndex < RX_NUM_CHANNELS)
        rxChannel[channelIndex++] = diff_us; // Save value in channel buffer
}

static void TimeoutHandler(void) {
    TimerIntClear(RX_LOST_TIMER_BASE, RX_LOST_INT_FLAG); // Clear interrupt
    writePPMAllOff(); // Turn all motors off
    validRXData = false; // Indicate that connection was lost
#ifndef DEBUG
    buzzer(true); // Turn on buzzer
#endif
}

// The first wide timer is used to measure the width of the pulses
// The second wide timer is used to turn off motors if the connection to the RX is lost
void initRX(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_RX_TIMER); // Enable timer peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_RX_LOST_TIMER); // Enable timer peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_RX); // Enable GPIO peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    GPIOPinConfigure(GPIO_RX_ALTERNATE); // Use alternate function
    GPIOPinTypeTimer(GPIO_RX_BASE, GPIO_RX); // Use pin with timer peripheral

    // Split timers and enable RX timer as event up-count timer and RX lost timer as a periodic timer
#if RX_TIMER_BASE == RX_LOST_TIMER_BASE
    TimerConfigure(RX_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | RX_TIMER_CFG | RX_LOST_TIMER_CFG);
#else
    TimerConfigure(RX_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | RX_TIMER_CFG);
    TimerConfigure(RX_LOST_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | RX_LOST_TIMER_CFG);
#endif

    // Configure RX timer
    TimerControlEvent(RX_TIMER_BASE, RX_TIMER, TIMER_EVENT_POS_EDGE); // Interrupt on positive edges
    TimerIntRegister(RX_TIMER_BASE, RX_TIMER, CaptureHandler); // Register interrupt handler
    TimerIntEnable(RX_TIMER_BASE, RX_INT_FLAG); // Enable timer capture event interrupt
    IntPrioritySet(RX_TIMER_INT, 0); // Configure timer interrupt priority as 0
    IntEnable(RX_TIMER_INT); // Enable timer interrupt

    // Configure RX lost timer
    timerLoadValue = SysCtlClockGet() / 10 - 1; // Set to interrupt every 100ms
    TimerLoadSet(RX_LOST_TIMER_BASE, RX_LOST_TIMER, timerLoadValue);
    TimerIntRegister(RX_LOST_TIMER_BASE, RX_LOST_TIMER, TimeoutHandler); // Register interrupt handler
    TimerIntEnable(RX_LOST_TIMER_BASE, RX_LOST_INT_FLAG); // Enable timer timeout interrupt
    IntPrioritySet(RX_LOST_TIMER_INT, 0); // Configure timer interrupt priority as 0
    IntEnable(RX_LOST_TIMER_INT); // Enable timer interrupt

    // Enable both timers
#if RX_TIMER_BASE == RX_LOST_TIMER_BASE
    TimerEnable(RX_TIMER_BASE, RX_TIMER | RX_LOST_TIMER);
#else
    TimerEnable(RX_TIMER_BASE, RX_TIMER);
    TimerEnable(RX_LOST_TIMER_BASE, RX_LOST_TIMER);
#endif

    validRXData = false;
}

// Returns the specific channel in the range [-100:100]
float getRXChannel(rxChannel_e channel) {
    return mapf(rxChannel[channel], RX_MIN_INPUT, RX_MAX_INPUT, -100.0f, 100.0f);
}
