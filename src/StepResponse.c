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
#include <stdlib.h>

#if STEP_ACRO_SELF_LEVEL || STEP_ALTITUDE_HOLD || STEP_HEADING_HOLD

#include "Pins.h"
#include "StepResponse.h"
#include "Time.h"
#include "uartstdio1.h" // Add "UART_BUFFERED1" to preprocessor - it uses a modified version of uartstdio, so it can be used with another UART interface

#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

typedef struct {
    uint32_t counter;
    uint32_t timeStamp;
    float setpoint;
    float input;
} step_t;

static void logData(step_t *step) {
    UARTprintf1("%u,%u,%d.%02u,%d.%02u\n",
                                        step->counter,
                                        step->timeStamp,
                                        (int16_t)step->setpoint, (uint16_t)(abs(step->setpoint * 100.0f) % 100),
                                        (int16_t)step->input, (uint16_t)(abs(step->input * 100.0f) % 100));
    UARTFlushTx1(false);
}

float stepResponse(bool active, float setpoint, float input, float step1, float step2, uint32_t interval, uint32_t now) {
    static uint8_t state;

    if (active) {
        static step_t step;
        static uint32_t startTime, stateTimer;

        switch (state) {
            case 0:
                startTime = stateTimer = now; // Set initial value
                step.counter = 0; // Reset counter
                setpoint = step1;
                state = 1;
                break;
            case 1:
                setpoint = step1;
                if ((int32_t)(now - stateTimer) >= interval) {
                    stateTimer = now;
                    state = 2;
                }
                break;
            case 2:
                setpoint = step2;
                if ((int32_t)(now - stateTimer) >= interval) {
                    stateTimer = now;
                    state = 3;
                }
                break;
            case 3:
                setpoint = step1;
                if ((int32_t)(now - stateTimer) >= interval)
                    state = 4;
                break;
            case 4:
                // Do nothing!
                break;
            default:
                break;
        }

        if (state < 4) { // Log data if state machine is running
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED

            step.counter++;
            step.timeStamp = now - startTime;
            step.setpoint = setpoint;
            step.input = input;

            logData(&step);
        } else
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED
    } else {
        state = 0;
        GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED
    }

    return setpoint;
}

#endif
