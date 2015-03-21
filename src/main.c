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
#include <stdlib.h>

#include "RX.h"
#include "UART.h"
#include "time.h"
#include "PPM.h"
#include "PID.h"
#include "MPU6500.h"
#include "sonar.h"

#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

#define SYSCTL_PERIPH_LED SYSCTL_PERIPH_GPIOF
#define GPIO_LED_BASE     GPIO_PORTF_BASE
#define GPIO_RED_LED      GPIO_PIN_1
#define GPIO_BLUE_LED     GPIO_PIN_2
#define GPIO_GREEN_LED    GPIO_PIN_3

int main(void) {
    // Set the clocking to run directly from the external crystal/oscillator.
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set clock to 80MHz (400MHz(PLL) / 2 / 2.5 = 80 MHz)

    initPPM();
    initUART();
    delay(100);
    UARTprintf("Started\n");
    initTime();
    initRX();
    initSonar();

    //initMPU6500();
    initMPU6500_i2c();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_LED); // Enable GPIOF peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinTypeGPIOOutput(GPIO_LED_BASE, GPIO_RED_LED | GPIO_BLUE_LED | GPIO_GREEN_LED); // Set red, blue and green LEDs as outputs

    IntMasterEnable();

    delay(100); // Wait a little for everything to initialize

    UARTprintf("CLK %d\n", SysCtlClockGet());
    UARTprintf("min: %d, max: %d, period: %d\n", PPM_MIN, PPM_MAX, getPeriod());

    pidRoll.Kp = 0.012f;
    pidRoll.Ki = 0.050f;
    pidRoll.Kd = 0.0f;

    pidRoll.integratedError = 0.0f;
    pidRoll.lastError = 0.0f;

    pidPitch = pidRoll; // Use same PID values for both pitch and roll

    // x2 the values work pretty well - TODO: Fine-tune these
    pidYaw = pidRoll;
    pidYaw.Kp *= 3.0f;
    pidYaw.Ki *= 3.5f; // I increased this in order for it to stop yawing slowly
    pidYaw.Kd *= 2.0f;

    printPIDValues();
    
    static const float angleKp = 60.0f;
    static const uint8_t stickScalingRollPitch = 30, stickScalingYaw = 30;
    static const float restAngleRoll = 3.62f, restAnglePitch = 0.20f;
    static const uint8_t maxAngleInclination = 50.0f; // Max angle in self level mode

    static int16_t accData[3], gyroData[3];
    float roll, pitch;

    static uint32_t imuTimer = 0, pidTimer = 0;

    // Motor 0 is bottom right, motor 1 is top right, motor 2 is bottom left and motor 3 is top left
    static float motors[4] = { -100.0f, -100.0f, -100.0f, -100.0f };

    static bool armed = false;

    while (!validRXData || rxChannel[RX_AUX2_CHAN] > RX_MID_INPUT) {
        // Wait until we have valid data and we are unarmed
    }

    while (1) {
        checkUARTData();

        // Make sure there is valid data, AUX channel is armed and that throttle is applied
        // The throttle check can be removed if one prefer the motors to spin once it is armed
        // TODO: Arm using throttle low and yaw right
        if (!validRXData || rxChannel[RX_AUX2_CHAN] < RX_MID_INPUT || rxChannel[RX_THROTTLE_CHAN] < RX_MIN_INPUT + 25) {
            writePPMAllOff();
            pidRoll.integratedError = pidRoll.lastError = 0.0f;
            pidPitch.integratedError = pidPitch.lastError = 0.0f;
            pidYaw.integratedError = pidYaw.lastError = 0.0f;
            armed = false;
        } else
            armed = true;

        // Turn on red led if there is valid data and AUX channel is in armed position otherwise turn on green LED
        GPIOPinWrite(GPIO_LED_BASE, GPIO_RED_LED | GPIO_GREEN_LED, !validRXData || rxChannel[RX_AUX2_CHAN] < RX_MID_INPUT ? GPIO_GREEN_LED : GPIO_RED_LED);
        
        uint32_t now = micros();
        if (dataReadyMPU6500()) {
            float dt = (float)(now - imuTimer) / 1000000.0f;
            //UARTprintf("%d\n", now - imuTimer);
            imuTimer = now;

            getMPU6500Data(accData, gyroData); // Get accelerometer and gyroscope values
            getMPU6500Angles(accData, gyroData, &roll, &pitch, dt); // Calculate pitch and roll

            roll -= restAngleRoll; // Apply angle trim
            pitch -= restAnglePitch;

            /*UARTprintf("%d\t%d\t%d\n", gyroData[0], gyroData[1], gyroData[2]);
            UARTFlushTx(false);*/
            /*UARTprintf("%d.%02d\t%d.%02d\n", (int16_t)roll, (int16_t)abs(roll * 100.0f) % 100, (int16_t)pitch, (int16_t)abs(pitch * 100.0f) % 100);
            UARTFlushTx(false);*/
        }

        bool angleMode;
        if (rxChannel[RX_AUX1_CHAN] > RX_MID_INPUT) {
            angleMode = true;
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED if in angle mode
        } else {
            angleMode = false;
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED if in acro mode
        }

        now = micros();
        float dt = (float)(now - pidTimer);
        if (armed && dt > 2500) { // Limit to 2.5ms (400 Hz)
            //UARTprintf("%d\n", now - pidTimer);
            pidTimer = now;
            dt /= 1000000.0f; // Convert to seconds

            float aileron = getRXChannel(RX_AILERON_CHAN);
            float elevator = getRXChannel(RX_ELEVATOR_CHAN);
            float rudder = getRXChannel(RX_RUDDER_CHAN);
            //UARTprintf("%d\t%d\t%d\n", (int16_t)aileron, (int16_t)elevator, (int16_t)rudder);

            float setPoint[2];
            if (angleMode) { // Angle mode
                setPoint[0] = constrain(aileron, -maxAngleInclination, maxAngleInclination) - roll;
                setPoint[1] = constrain(elevator, -maxAngleInclination, maxAngleInclination) - pitch;
                setPoint[0] *= angleKp;
                setPoint[1] *= angleKp;
            } else { // Acro mode
                setPoint[0] = aileron * stickScalingRollPitch;
                setPoint[1] = elevator * stickScalingRollPitch;
            }
            
            UARTprintf("%d\t%d\n", (int16_t)setPoint[0], (int16_t)setPoint[1]);
            UARTFlushTx(false);

            float rollOut = updatePID(&pidRoll, setPoint[0], gyroData[1], dt);
            float pitchOut = updatePID(&pidPitch, setPoint[1], gyroData[0], dt);

            float yawOut = updatePID(&pidYaw, rudder * stickScalingYaw, gyroData[2], dt);

            float throttle = getRXChannel(RX_THROTTLE_CHAN);
            for (uint8_t i = 0; i < 4; i++)
                motors[i] = throttle;

            motors[0] -= rollOut;
            motors[1] -= rollOut;
            motors[2] += rollOut;
            motors[3] += rollOut;

            motors[0] += pitchOut;
            motors[1] -= pitchOut;
            motors[2] += pitchOut;
            motors[3] -= pitchOut;

            motors[0] -= yawOut;
            motors[1] += yawOut;
            motors[2] += yawOut;
            motors[3] -= yawOut;

            updateMotorsAll(motors);

            //UARTprintf("%d\t%d\n", (int16_t)elevator, (int16_t)aileron);
#if 0
            UARTprintf("%d\t%d\t\t", (int16_t)roll, (int16_t)pitch);
            UARTprintf("%d\t%d\t\t", (int16_t)rollOut, (int16_t)pitchOut);
            UARTprintf("%d\t%d\t%d\t%d\n", (int16_t)motors[0], (int16_t)motors[1], (int16_t)motors[2], (int16_t)motors[3]);
            UARTFlushTx(false);
#endif
        }

        triggerSonar(); // Trigger sonar
    }
}

// TODO:
    // Save PID values in EEPROM
    // Adjust PID values using pots on transmitter
    // Only enable peripheral clock once
    // Tune yaw PID values separately
    // Make limit of integrated error adjustable
    // Use SPI instead of I2C
    // Set Kd as well
    // Scope PWM output and check that it is in sync with control loop
    // Define all pins in a pins.h
    // Rename sonar.h to Sonar.h and time.h to Time.h
    // Update year in copyright header
    // Use sonar distance for something usefull - see: https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/altitudehold.c
        // https://github.com/cleanflight/cleanflight/blob/master/src/main/sensors/sonar.c#L90-L99
    // Limit other motors if one reaches maximum: https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/mixer.c#L677-L684
    // Make acc calibration routine - apply zero values before Kalman routine
    // Take average of several gyro readings for calibration routine
    // Retune PID again and tune stickscaling
    // TOOD: Read gyro values multiple times and check if it's moved while doing so
    // Only have one Kalman.c file. Use struct as argument instead
    // Takes average of three readings in DTerm: https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/pid.c#L721-L732
