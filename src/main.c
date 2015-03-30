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

#include "Bluetooth.h"
#include "EEPROM.h"
#include "I2C.h"
#include "MPU6500.h"
#include "PPM.h"
#include "PID.h"
#include "RX.h"
#include "Sonar.h"
#include "Time.h"
#include "UART.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

#define SYSCTL_PERIPH_LED SYSCTL_PERIPH_GPIOF
#define GPIO_LED_BASE     GPIO_PORTF_BASE
#define GPIO_RED_LED      GPIO_PIN_1
#define GPIO_BLUE_LED     GPIO_PIN_2
#define GPIO_GREEN_LED    GPIO_PIN_3

static mpu6500_t mpu6500; // Gyro and accelerometer readings
static float roll, pitch; // Roll and pitch calculated using Kalman filter
static uint32_t imuTimer = 0, pidTimer = 0; // Used to keep track of the time

// Motor 0 is bottom right, motor 1 is top right, motor 2 is bottom left and motor 3 is top left
static float motors[4] = { -100.0f, -100.0f, -100.0f, -100.0f };
static bool armed = false;

int main(void) {
    // Set the clocking to run directly from the external crystal/oscillator and use PLL to run at 80 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set clock to 80 MHz (400 MHz(PLL) / 2 / 2.5 = 80 MHz)

    initUART();
    initEEPROM();
    initTime();
    initPPM();
    initRX();
    initSonar();
    initI2C();
    initMPU6500();
    initBluetooth();
    IntMasterEnable(); // Enable all interrupts

    SysCtlPeripheralEnable(SYSCTL_PERIPH_LED); // Enable GPIOF peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinTypeGPIOOutput(GPIO_LED_BASE, GPIO_RED_LED | GPIO_BLUE_LED | GPIO_GREEN_LED); // Set red, blue and green LEDs as outputs

    setDefaultPIDValues();
    printPIDValues();

    while (!validRXData || getRXChannel(RX_AUX2_CHAN) > 0) {
        // Wait until we have valid data and safety aux channel is in safe position
    }

#if 0 // Set to one in order to run the acceleromter calibration routine
    while (calibrateAcc()) { // Get accelerometer zero values
        // Loop until calibration values are found
    }
#else
    UARTprintf("Accelerometer zero values: %d\t%d\t%d\n", cfg.accZero.X, cfg.accZero.Y, cfg.accZero.Z);
#endif

#if 0 // Set to one in order to run the ESC calibration routine at next power cycle
    // WARNING: Do this with propellers off!!
    // Also set CALIBRATE_ESC_ACTIVATED to 1 inside PPM.c
    calibrateESCs(true); // ESCs will be calibrated on next power cycle
    UARTprintf("Calibrating ESCs on next power cycle\n");
#endif

    while (1) {
        // Make sure there is valid data and safety channel is in armed position
        if (validRXData && getRXChannel(RX_AUX2_CHAN) > 0) {
            if (!armed && getRXChannel(RX_THROTTLE_CHAN) < -95 && getRXChannel(RX_RUDDER_CHAN) > 95) // Arm using throttle low and yaw right
                armed = true;
            else if (armed && getRXChannel(RX_THROTTLE_CHAN) < -95 && getRXChannel(RX_RUDDER_CHAN) < -95) // Disarm using throttle low and yaw left
                armed = false;
        } else
            armed = false;

        // Turn on red led if armed otherwise turn on green LED
        GPIOPinWrite(GPIO_LED_BASE, GPIO_RED_LED | GPIO_GREEN_LED, armed ? GPIO_RED_LED : GPIO_GREEN_LED);

        uint32_t now = micros();
        if (dataReadyMPU6500()) {
            float dt = (float)(now - imuTimer) / 1000000.0f;
            //UARTprintf("%d\n", now - imuTimer);
            imuTimer = now;

            getMPU6500Data(&mpu6500); // Get accelerometer and gyroscope values
            getMPU6500Angles(&mpu6500, &roll, &pitch, dt); // Calculate pitch and roll

            /*UARTprintf("%d\t%d\t%d\n", mpu6500.gyro.X, mpu6500.gyro.Y, mpu6500.gyro.Z);
            UARTFlushTx(false);*/
            /*UARTprintf("%d.%02d\t%d.%02d\n", (int16_t)roll, (int16_t)abs(roll * 100.0f) % 100, (int16_t)pitch, (int16_t)abs(pitch * 100.0f) % 100);
            UARTFlushTx(false);*/
        }

        static bool angleMode = false;
        if (!angleMode && getRXChannel(RX_AUX1_CHAN) > -10) {
            angleMode = true;
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED if in angle mode
        } else if (angleMode) {
            angleMode = false;
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED if in acro mode
        }

        // Don't spin motors if the throttle is low
        bool runMotors = false;
        if (armed && getRXChannel(RX_THROTTLE_CHAN) > -95)
            runMotors = true;
        else {
            writePPMAllOff();
            resetPIDError();
        }
        
        if (!armed)
            checkUARTData(); // Poll UART for incoming data if unarmed

        if (!runMotors)
            readBluetooth(); // Read Bluetooth data if motors are not spinning

        if (runMotors) {
            now = micros();
            float dt = (float)(now - pidTimer);
            if (runMotors && dt > 2500) { // Limit to 2.5ms (400 Hz)
                //UARTprintf("%d\n", now - pidTimer);
                pidTimer = now;
                dt /= 1000000.0f; // Convert to seconds

                float aileron = getRXChannel(RX_AILERON_CHAN);
                float elevator = getRXChannel(RX_ELEVATOR_CHAN);
                float rudder = getRXChannel(RX_RUDDER_CHAN);
                //UARTprintf("%d\t%d\t%d\n", (int16_t)aileron, (int16_t)elevator, (int16_t)rudder);

                float setPointRoll, setPointPitch; // Roll and pitch control can both be gyro or accelerometer based
                const float setPointYaw = rudder * cfg.stickScalingYaw; // Yaw is always gyro controlled
                if (angleMode) { // Angle mode
                    setPointRoll = constrain(aileron, -cfg.maxAngleInclination, cfg.maxAngleInclination) - roll;
                    setPointPitch = constrain(elevator, -cfg.maxAngleInclination, cfg.maxAngleInclination) - pitch;
                    setPointRoll *= cfg.angleKp;
                    setPointPitch *= cfg.angleKp;
                } else { // Acro mode
                    setPointRoll = aileron * cfg.stickScalingRollPitch;
                    setPointPitch = elevator * cfg.stickScalingRollPitch;
                }

                /*UARTprintf("%d\t%d\n", (int16_t)setPointRoll, (int16_t)setPointPitch);
                UARTFlushTx(false);*/

                float rollOut = updatePID(&cfg.pidRoll, setPointRoll, mpu6500.gyroRate.Y, dt);
                float pitchOut = updatePID(&cfg.pidPitch, setPointPitch, mpu6500.gyroRate.X, dt);
                float yawOut = updatePID(&cfg.pidYaw, setPointYaw, mpu6500.gyroRate.Z, dt);

                float throttle = getRXChannel(RX_THROTTLE_CHAN);
                for (uint8_t i = 0; i < 4; i++)
                    motors[i] = throttle;

                // Apply mix for quadcopter in x-configuration
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
        }

        triggerSonar(); // Trigger sonar
    }
}

// TODO:
    // Adjust PID values using pots on transmitter
    // Only enable peripheral clock once
    // Use SPI instead of I2C for MPU-6500
    // Scope PWM output and check that it is in sync with control loop
    // Define all pins in a pins.h
    // Use sonar distance for something usefull - see: https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/altitudehold.c
        // https://github.com/cleanflight/cleanflight/blob/master/src/main/sensors/sonar.c#L90-L99
    // Limit other motors if one reaches maximum: https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/mixer.c#L677-L684
    // Create Android App
        // Tune yaw PID values separately
        // Set Kd as well
        // Retune PID again and tune stickscaling
    // Takes average of three readings in DTerm: https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/pid.c#L721-L732
    // Add buzzer. Beep on startup, arm changed, turn on at gyro/acc calibration error, connection loss etc.
    // Add disarm timer
    // Remove safety AUX channel once 100% stable
    // Measure loop time - print value or toggle I/O pin
    // Check that both buttons are held in while calibrating ESCs
