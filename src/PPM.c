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

#include "Buzzer.h"
#include "EEPROM.h"
#include "PID.h"
#include "PPM.h"
#include "Time.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

#define CALIBRATE_ESC_ACTIVATED 0
#define ONESHOT125 1

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
void calibrateESCs(bool flag) {
    cfg.calibrateESCs = flag; // Set flag
    updateConfig(); // Write new value to EEPROM
}

void initPPM(void) {
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

    if (cfg.calibrateESCs) {
#if CALIBRATE_ESC_ACTIVATED
        #warning "Take propellers off and then only apply power from the battery once and then unplug the battery after calibration procedure is done!"
        // ESCs are calibrated by sending out the maximum pulse when power is applied and then sending lowest pulse afterwards
        for (uint8_t i = 0; i < 4; i++)
            writePPMUs(i, PPM_MAX);
#if ONESHOT125
        syncMotors();
#endif
        delay(4000); // Wait 4s - TODO: Make sure both buttons are held in
        for (uint8_t i = 0; i < 4; i++)
            writePPMUs(i, PPM_MIN);
#if ONESHOT125
        syncMotors();
#endif
        delay(3000);
        buzzerLongBeep();
#else // CALIBRATE_ESC_ACTIVATED
        writePPMAllOff();
#endif
        calibrateESCs(false); // Set back to false

        while (1) {
            // Prevent user from flying
        }
    } else
        writePPMAllOff();

#if ONESHOT125
    syncMotors();
#endif
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
    for (uint8_t i = 0; i < 4; i++)
        updateMotor(i, values[i]);
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
