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
#include "EEPROM.h"
#include "PID.h"
#include "Pins.h"
#include "PPM.h"
#include "Time.h"

#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#define SYSCTL_PERIPH_SW    SYSCTL_PERIPH_GPIOF
#define GPIO_SW_BASE        GPIO_PORTF_BASE
#define GPIO_SW1            GPIO_PIN_4
#define GPIO_SW2            GPIO_PIN_0

#ifndef ONESHOT125
#define ONESHOT125 1
#endif

#if ONESHOT125
    #define PPM_MIN 1250 // The values in OneShot125 are in 0.1 us steps to increase the resolution
    #define PPM_MAX 2500
#else
    #define PPM_MIN 1064 // From SimonK firmware
    #define PPM_MAX 1864 // From SimonK firmware
#endif

static uint16_t period;

static void writePPMUs(uint8_t motor, uint16_t us);
#if ONESHOT125
static void syncMotors(void);
#endif

// Sets calibrating flag in EEPROM
// Calibration routine will be run next time power is applied if flag is true
static void calibrateESCs(bool flag) {
    cfg.calibrateESCs = flag; // Set flag
    updateConfig(); // Write new value to EEPROM
}

void initPPM(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SW); // Enable peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
#if defined(PART_TM4C123GH6PM) && SYSCTL_PERIPH_SW == SYSCTL_PERIPH_GPIOF && (GPIO_SW1 == GPIO_PIN_0 || GPIO_SW2 == GPIO_PIN_0)
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // Unlocks the GPIO_CR register
    GPIO_PORTF_CR_R |= GPIO_PIN_0; // Allow changes to PF0
    GPIO_PORTF_LOCK_R = 0; // Lock register again
#endif
    GPIOPinTypeGPIOInput(GPIO_SW_BASE, GPIO_SW1 | GPIO_SW2); // Set both switches as inputs
    GPIOPadConfigSet(GPIO_SW_BASE, GPIO_SW1 | GPIO_SW2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // Turn on pull-ups

#if ONESHOT125
    SysCtlPWMClockSet(SYSCTL_PWMDIV_2); // Set divider to 2
#else
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4); // Set divider to 4
#endif

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    // Use alternate function
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5); // Use pin with PWM peripheral

    // Configure the PWM generator for count down mode with immediate updates to the parameters
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // The value is given by (SysClk * period) / divider
#if ONESHOT125
    // The period is set to 1.5 ms (666 Hz) - this just has to be longer than the IMU update rate (1 kHz)
    period = (SysCtlClockGet() / 10000 * 15) / 2; // = 60000
#else
    // The period is set to 2.5ms (400 Hz)
    period = (SysCtlClockGet() / 10000 * 25) / 4; // = 50000
#endif
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);

    // Start the timers in generator 0 and 1
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    // Enable the outputs
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);

    writePPMAllOff();

    if (GPIOPinRead(GPIO_SW_BASE, GPIO_SW1 | GPIO_SW2) == 0) { // Check if both switches are pressed
        if (cfg.calibrateESCs) { // Check if arming flag is set
            calibrateESCs(false); // Set back to false
            uint32_t resetCause = SysCtlResetCauseGet();
            SysCtlResetCauseClear(resetCause); // Clear all reset causes
            if (resetCause == SYSCTL_CAUSE_POR) { // Make sure that reset was caused by a power on reset only
                GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED
#if UART_DEBUG
                UARTprintf("Calibrating ESCs!\nSending out maximum pulse\n");
#endif
                // ESCs are calibrated by sending out the maximum pulse when power is applied and then sending lowest pulse afterwards
                for (uint8_t i = 0; i < 4; i++)
                    writePPMUs(i, PPM_MAX);
#if ONESHOT125
                syncMotors();
#endif
                uint32_t start = micros();
                while ((int32_t)(micros() - start) < 6000*1000UL) { // Wait 6s
                    if (GPIOPinRead(GPIO_SW_BASE, GPIO_SW1 | GPIO_SW2) != 0) { // Abort if the user releases any of the switches
#if UART_DEBUG
                        UARTprintf("Aborting ESC calibration!\n");
#endif
                        writePPMAllOff(); // Turn off motors
                        buzzer(true); // Turn on buzzer
                        while (1) {
                            // Prevent user from flying
                        }
                    }
                };
#if UART_DEBUG
                UARTprintf("Sending out minimum pulse\n");
#endif
                for (uint8_t i = 0; i < 4; i++)
                    writePPMUs(i, PPM_MIN);
#if ONESHOT125
                syncMotors();
#endif
                delay(6000); // Wait 6s
#if UART_DEBUG
                UARTprintf("Calibrating of the ESCs was successful\n");
#endif
                GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED
                beepLongBuzzer();
                while (1) {
                    // Prevent user from flying
                }
            }
#if UART_DEBUG
            else
                UARTprintf("Calibration aborted. Reset was not due to a power on reset!\n");
#endif
        } else {
#if UART_DEBUG
            UARTprintf("Calibrating ESCs on next power cycle\nTake propellers off and then apply power from the battery while holding the two hardware switches\nUnplug the battery when the calibration procedure is done\n");
#endif
            calibrateESCs(true); // ESCs will be calibrated on next power cycle
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED
            while (1) {
                // Prevent user from flying
            }
        }
    } else if (cfg.calibrateESCs) { // Clear ESC calibration flag
#if UART_DEBUG
        UARTprintf("Calibrating of ESCs canceled!\n");
#endif
        calibrateESCs(false);
    }
}

// Turn off all motors
void writePPMAllOff(void) {
    for (uint8_t i = 0; i < 4; i++)
        writePPMUs(i, PPM_MIN);
#if ONESHOT125
    syncMotors();
#endif
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    float value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; // From Arduino source code: https://github.com/arduino/Arduino/blob/ide-1.5.x/hardware/arduino/avr/cores/arduino/WMath.cpp
    return constrain(value, out_min, out_max); // Limit output
}

// Motor input is in the range [MIN_MOTOR_OUT:MAX_MOTOR_OUT]
static void updateMotor(uint8_t motor, float value) {
    uint16_t motorOutput = mapf(value, MIN_MOTOR_OUT, MAX_MOTOR_OUT, PPM_MIN, PPM_MAX); // Map to PPM min and max value
    writePPMUs(motor, motorOutput);
}

void updateMotorsAll(float *values) {
    // Find the maximum motor output
    float maxMotor = values[0];
    for (uint8_t i = 1; i < 4; i++) {
        // If one motor is above the maxthrottle threshold, we reduce the value
        // of all motors by the amount of overshoot. That way, only one motor
        // is at max and the relative power of each motor is preserved
        if (values[i] > maxMotor)
            maxMotor = values[i];
    }

    if (maxMotor > MAX_MOTOR_OUT) {
        for (uint8_t i = 0; i < 4; i++)
            values[i] -= maxMotor - MAX_MOTOR_OUT; // This is a way to still have good gyro corrections if at least one motor reaches its max
    }

    for (uint8_t i = 0; i < 4; i++)
        updateMotor(i, values[i]); // Write output to ESCs
#if ONESHOT125
    syncMotors();
#endif
}

static void writePPMWidth(uint8_t motor, uint16_t width) {
    PWMPulseWidthSet(PWM0_BASE, motor == 0 ? PWM_OUT_0 : motor == 1 ? PWM_OUT_1 : motor == 2 ? PWM_OUT_2 : PWM_OUT_3, width);
}

static void writePPMUs(uint8_t motor, uint16_t us) {
#if ONESHOT125
    writePPMWidth(motor, period * us / 15000); // 666 Hz - the values in OneShot125 are in 0.1 us steps to increase the resolution
#else
    writePPMWidth(motor, period * us / 2500); // 400 Hz
#endif
}

#if ONESHOT125
static void syncMotors(void) { // Used to update the output values immediately ignoring the period
    PWMSyncUpdate(PWM0_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT); // Apply any updates of the pulse width next time generator counters becomes 0
    PWMSyncTimeBase(PWM0_BASE, PWM_GEN_0_BIT | PWM_GEN_1_BIT); // Reset both generator counters to 0, so the width value is written to the output
}
#endif
