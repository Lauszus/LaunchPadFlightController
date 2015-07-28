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
#include <math.h>

#include "AltitudeHold.h"
#include "Bluetooth.h"
#include "Buzzer.h"
#include "EEPROM.h"
#include "HeadingHold.h"
#include "I2C.h"
#include "IMU.h"
#include "Magnetometer.h"
#include "MPU6500.h"
#include "PPM.h"
#include "PID.h"
#include "Pins.h"
#include "RX.h"
#include "StepResponse.h"
#include "Time.h"
#include "UART.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

static angle_t angle; // Struct used to store angles
static mpu6500_t mpu6500; // Gyro and accelerometer readings
static sensor_t mag = { .data = { 1.0f, 0.0f, 0.0f } }; // If no magnetometer is used, then just use a vector with a x-component only

int main(void) {
    // Set the clocking to run directly from the external crystal/oscillator and use PLL to run at 80 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set clock to 80 MHz (400 MHz(PLL) / 2 / 2.5 = 80 MHz)

    initPID();
#if UART_DEBUG
    initUART();
#endif
    initTime();
    initBuzzer();
    initEEPROM();
    initPPM();
    initRX();
    initI2C();
    initMPU6500(&mpu6500);
#if USE_MAG
    initMag();
#endif
#if USE_SONAR || USE_BARO
    initAltitudeHold();
#endif
    initBluetooth();
    IntMasterEnable(); // Enable all interrupts

    SysCtlPeripheralEnable(SYSCTL_PERIPH_LED); // Enable peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinTypeGPIOOutput(GPIO_LED_BASE, GPIO_RED_LED | GPIO_BLUE_LED | GPIO_GREEN_LED); // Set red, blue and green LEDs as outputs

#if UART_DEBUG
    UARTprintf("Accelerometer zero values: %d\t%d\t%d\n", cfg.accZero.axis.X, cfg.accZero.axis.Y, cfg.accZero.axis.Z);
#endif

#if UART_DEBUG && USE_MAG
    UARTprintf("Magnetometer zero values: %d\t%d\t%d\n", (int16_t)cfg.magZero.axis.X, (int16_t)cfg.magZero.axis.Y, (int16_t)cfg.magZero.axis.Z);
#endif

#if 0 // Set to 1 in order to run the ESC calibration routine at next power cycle
    // WARNING: Do this with propellers off!!
    // Also set CALIBRATE_ESC_ACTIVATED to 1 inside PPM.c
    calibrateESCs(true); // ESCs will be calibrated on next power cycle
    UARTprintf("Calibrating ESCs on next power cycle\n");
#endif

#if UART_DEBUG
    printPIDValues(pidRoll.values); // Print PID Values
    printPIDValues(pidYaw.values);
    printPIDValues(pidAltHold.values);
    printSettings(); // Print settings
#endif

    while (!validRXData) {
        // Wait until we have valid data
    }

#if UART_DEBUG
    UARTprintf("Ready\n");
#endif
    beepBuzzer(); // Indicate startup

    while (1) {
        // Make sure there is valid data and safety channel is in armed position
        static bool armed = false;
        if (validRXData) {
            if (!armed && getRXChannel(RX_THROTTLE_CHAN) < CHANNEL_MIN_CHECK && getRXChannel(RX_RUDDER_CHAN) > CHANNEL_MAX_CHECK) // Arm using throttle low and yaw right
                armed = true;
            else if (armed && getRXChannel(RX_THROTTLE_CHAN) < CHANNEL_MIN_CHECK && getRXChannel(RX_RUDDER_CHAN) < CHANNEL_MIN_CHECK) // Disarm using throttle low and yaw left
                armed = false;
        } else
            armed = false;

        static bool lastArmed = false;
        if (armed != lastArmed)
            beepBuzzer(); // Indicate that armed status were changed
        lastArmed = armed;

        // Turn on red led if armed otherwise turn on green LED
        GPIOPinWrite(GPIO_LED_BASE, GPIO_RED_LED | GPIO_GREEN_LED, armed ? GPIO_RED_LED : GPIO_GREEN_LED);

        // Handle the different modes
        bool angleMode = getRXChannel(RX_AUX1_CHAN) > -10;
#if USE_MAG
        bool headMode = angleMode && getRXChannel(RX_AUX1_CHAN) > 50; // Make sure angle mode is activated in heading hold mode
#endif
#if USE_SONAR || USE_BARO
        bool altitudeMode = angleMode && getRXChannel(RX_AUX2_CHAN) > 0; // Make sure angle mode is activated in altitude hold mode
#endif

        // Don't spin motors if the throttle is low
        bool runMotors = false;
        if (armed &&
#if USE_SONAR
                (getRXChannel(RX_THROTTLE_CHAN) > CHANNEL_MIN_CHECK || altitudeMode)) // If in altitude mode, keep motors spinning anyway
#else
                getRXChannel(RX_THROTTLE_CHAN) > CHANNEL_MIN_CHECK)
#endif
            runMotors = true;
        else if (readBluetoothData(&mpu6500, &angle)) // Read Bluetooth data if motors are not spinning
            beepBuzzer(); // Indicate if new values were set

        if (dataReadyMPU6500()) {
            uint32_t now = micros();
            static uint32_t timer = 0; // Used to keep track of the time
            float dt = (float)(now - timer) / 1e6f; // Convert to seconds
            //UARTprintf("%d\n", now - timer);
            timer = now;

            // Read IMU
            getMPU6500Data(&mpu6500); // Get accelerometer and gyroscope values
#if USE_MAG
            getMagData(&mag, false); // Get magnetometer values with zero values subtracted
#endif
            getAngles(&mpu6500, &mag, &angle, dt); // Calculate pitch, roll and yaw

#if USE_SONAR || USE_BARO
            static altitude_t altitude;
            getAltitude(&angle, &mpu6500, &altitude, now, dt);
#endif

            /*UARTprintf("%d\t%d\t%d\n", (int16_t)angle.axis.roll, (int16_t)angle.axis.pitch, (int16_t)angle.axis.yaw);
            UARTFlushTx(false);*/

            // Motors routine
            if (runMotors) {
                float aileron = getRXChannel(RX_AILERON_CHAN);
                float elevator = getRXChannel(RX_ELEVATOR_CHAN);
                float rudder = getRXChannel(RX_RUDDER_CHAN);
                //UARTprintf("%d\t%d\t%d\n", (int16_t)aileron, (int16_t)elevator, (int16_t)rudder);

#if USE_MAG
                if (headMode && fabsf(rudder) < 5) // Only use heading hold if user is not applying rudder
                    rudder = updateHeadingHold(&angle, rudder, now);
                else
                    resetHeadingHold(&angle);
#endif

                float setPointRoll, setPointPitch; // Roll and pitch control can both be gyro or accelerometer based
                const float setPointYaw = rudder * cfg.stickScalingYaw; // Yaw is always gyro controlled
                if (angleMode) { // Angle mode
                    const uint8_t maxAngleInclination =
#if USE_SONAR
                            altitudeMode ? cfg.maxAngleInclinationSonar :
#endif
                            cfg.maxAngleInclination; // If in altitude mode the angle has to be limited to the capability of the sonar

#if STEP_ACRO_SELF_LEVEL
                    static const float step1 = 0; // Start at 0 degrees
                    static const float step2 = 15; // Tilt 15 degrees
                    static const uint32_t interval = 1e6; // 1s between steps
                    aileron = stepResponse(getRXChannel(RX_AUX2_CHAN) > 0, aileron, angle.axis.roll, step1, step2, interval, now);
#endif
                    setPointRoll = constrain(aileron, -maxAngleInclination, maxAngleInclination) - angle.axis.roll;
                    setPointPitch = constrain(elevator, -maxAngleInclination, maxAngleInclination) - angle.axis.pitch;
                    setPointRoll *= cfg.angleKp; // A cascaded P controller is used in self level mode, as the output from the P controller is then used as the set point for the acro PID controller
                    setPointPitch *= cfg.angleKp;
                } else { // Acro mode
                    setPointRoll = aileron * cfg.stickScalingRollPitch;
                    setPointPitch = elevator * cfg.stickScalingRollPitch;

#if STEP_ACRO_SELF_LEVEL
                    static const float step1 = 0; // Start at 0 degrees/s
                    static const float step2 = 15; // Rotate 15 degrees/s
                    static const uint32_t interval = 1e6; // 1s between steps
                    setPointRoll = stepResponse(getRXChannel(RX_AUX2_CHAN) > 0, setPointRoll, mpu6500.gyroRate.axis.roll, step1, step2, interval, now);
#endif
                }

                /*UARTprintf("%d\t%d\n", (int16_t)setPointRoll, (int16_t)setPointPitch);
                UARTFlushTx(false);*/

                float rollOut = updatePID(&pidRoll, setPointRoll, mpu6500.gyroRate.axis.roll, dt);
                float pitchOut = updatePID(&pidPitch, setPointPitch, mpu6500.gyroRate.axis.pitch, dt);
                float yawOut = updatePID(&pidYaw, setPointYaw, -mpu6500.gyroRate.axis.yaw, dt); // Gyro rate is inverted, so it works well with RC yaw control input

                float throttle = getRXChannel(RX_THROTTLE_CHAN);

#if USE_SONAR || USE_BARO
                if (altitudeMode)
                    throttle = updateAltitudeHold(&altitude, throttle, now, dt);
                else
                    resetAltitudeHold();
#endif

                float motors[4]; // Motor 0 is bottom right, motor 1 is top right, motor 2 is bottom left and motor 3 is top left
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
                UARTprintf("%d\t%d\t\t", (int16_t)angle.axis.roll, (int16_t)angle.axis.pitch);
                UARTprintf("%d\t%d\t\t", (int16_t)rollOut, (int16_t)pitchOut);
                UARTprintf("%d\t%d\t%d\t%d\n", (int16_t)motors[0], (int16_t)motors[1], (int16_t)motors[2], (int16_t)motors[3]);
                UARTFlushTx(false);
#endif
            } else {
                writePPMAllOff();
                resetPIDRollPitchYaw();
#if USE_SONAR || USE_BARO
                resetAltitudeHold();
#endif
#if USE_MAG
                resetHeadingHold(&angle);
#endif
            }
        }
#if 0
        static uint32_t loopTimer;
        while ((int32_t)(micros() - loopTimer) < 2500) {
            // Limit loop to 400 Hz
        }
        loopTimer = micros();
#endif
    }
}

// TODO:
    // Altitude hold
        // Implement altitude hold using height estimated using barometer
        // Use wide timer for sonar
        // Redo take off sequence
            // Ramp up motors slowly
    // Android App
        // Self level angle trim
        // Set magnetic declination
        // Set acc_lpf_factor, gyro_cmpf_factor, gyro_cmpfm_factor, baro_noise_lpf and throttle_noise_lpf + add explanation
        // Set headMaxAngle
        // Set altHoldSetPoint and altHoldInitialThrottle for altitude hold mode
        // Control drone using virtual joystick
            // Auto take off and land in altitude hold mode
        // Show distance in graph as well
    // Add disarm timer
    // Check that both buttons are held in while calibrating ESCs
    // All filters should depend on dt as well, so loop time does not affect them
        // And they should also be on the same form to make it consistent
    // Store angles in radians as well
    // IMU driver should have MPU-6500 and HMC5883L instances, so they did not have to be in the main loop
    // Make yaw right hand rotation
    // Move all IMU related code into IMU driver
        // Also make generic accGyro driver
    // Use pointer for magnetometer functions
