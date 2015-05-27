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

#if USE_BARO
#include "BMP180.h"
#endif
#include "Bluetooth.h"
#include "Buzzer.h"
#include "EEPROM.h"
#if USE_MAG
#include "HMC5883L.h"
#endif
#include "I2C.h"
#include "IMU.h"
#include "MPU6500.h"
#include "PPM.h"
#include "PID.h"
#include "Pins.h"
#include "RX.h"
#if USE_SONAR
#include "Sonar.h"
#endif
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
#if USE_MAG
static hmc5883l_t hmc5883l; // Magnetometer readings
#endif
#if USE_BARO
static bmp180_t bmp180; // Barometer readings
#endif

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
#if USE_SONAR
    initSonar();
#endif
    initI2C();
    initMPU6500(&mpu6500);
#if USE_MAG
    intHMC5883L(&hmc5883l);
#endif
#if USE_BARO
    intBMP180(&bmp180);
#endif
    initBluetooth();
    IntMasterEnable(); // Enable all interrupts

    SysCtlPeripheralEnable(SYSCTL_PERIPH_LED); // Enable peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinTypeGPIOOutput(GPIO_LED_BASE, GPIO_RED_LED | GPIO_BLUE_LED | GPIO_GREEN_LED); // Set red, blue and green LEDs as outputs

#if UART_DEBUG
    UARTprintf("Accelerometer zero values: %d\t%d\t%d\n", cfg.accZero.axis.X, cfg.accZero.axis.Y, cfg.accZero.axis.Z);
#endif

#if 0 && USE_MAG // Set this to 1 in order to run the magnetometer calibration routine
#if UART_DEBUG
    UARTprintf("Starting magnetometer calibration\n");
#endif
    calibrateMag(&hmc5883l);
#if UART_DEBUG
    UARTprintf("Finished magnetometer calibration: %d %d %d\n", (int16_t)cfg.magZero.axis.X, (int16_t)cfg.magZero.axis.Y, (int16_t)cfg.magZero.axis.Z);
#endif
#elif UART_DEBUG && USE_MAG
    UARTprintf("Magnetometer zero values: %d\t%d\t%d\n", (int16_t)cfg.magZero.axis.X, (int16_t)cfg.magZero.axis.Y, (int16_t)cfg.magZero.axis.Z);
#endif
#if UART_DEBUG && USE_BARO
    UARTprintf("Barometer values: %d\t%d\t%d\n", bmp180.pressure, bmp180.temperature, (int32_t)bmp180.groundAltitude);
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
            if (!armed && getRXChannel(RX_THROTTLE_CHAN) < -95 && getRXChannel(RX_RUDDER_CHAN) > 95) // Arm using throttle low and yaw right
                armed = true;
            else if (armed && getRXChannel(RX_THROTTLE_CHAN) < -95 && getRXChannel(RX_RUDDER_CHAN) < -95) // Disarm using throttle low and yaw left
                armed = false;
        } else
            armed = false;

        static bool lastArmed = false;
        if (armed != lastArmed)
            beepBuzzer(); // Indicate that armed status were changed
        lastArmed = armed;

        // Turn on red led if armed otherwise turn on green LED
        GPIOPinWrite(GPIO_LED_BASE, GPIO_RED_LED | GPIO_GREEN_LED, armed ? GPIO_RED_LED : GPIO_GREEN_LED);

        // Don't spin motors if the throttle is low
        bool runMotors = false;
        if (armed && getRXChannel(RX_THROTTLE_CHAN) > -95)
            runMotors = true;
        else {
            if (readBluetoothData(&mpu6500, &angle)) // Read Bluetooth data if motors are not spinning
                beepBuzzer(); // Indicate if new values were set
        }

        // Handle angle and heading mode
        bool angleMode = false;
        if (getRXChannel(RX_AUX1_CHAN) > -10)
            angleMode = true;

#if USE_MAG
        if (!armed)
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED if not armed
#endif

        if (dataReadyMPU6500()) {
            uint32_t now = micros();
            static uint32_t timer = 0; // Used to keep track of the time
            float dt = (float)(now - timer) / 1000000.0f;
            //UARTprintf("%d\n", now - timer);
            timer = now;

            // Read IMU
            getMPU6500Data(&mpu6500); // Get accelerometer and gyroscope values
#if USE_MAG
            if (dataReadyHMC5883L()) // The HMC5883L update rate is very slow (15 Hz), so it does not matter that we sample inside here
                getHMC5883LData(&hmc5883l, false); // Get magnetometer values with zero values subtracted
            getAngles(&mpu6500, &hmc5883l.mag, &angle, dt); // Calculate pitch, roll and yaw
#else
            // If no magnetometer is used, then just define a vector with a x-component only
            static sensor_t mag = { .data = { 1.0f, 0.0f, 0.0f } };
            getAngles(&mpu6500, &mag, &angle, dt); // Calculate pitch, roll and yaw
#endif

            /*UARTprintf("%d\t%d\t%d\n", (int16_t)angle.axis.roll, (int16_t)angle.axis.pitch, (int16_t)angle.axis.yaw);
            UARTFlushTx(false);*/

            // Motors routine
#if USE_MAG
            static float magHold; // Heading using for heading hold
#endif
            if (runMotors) {
                float aileron = getRXChannel(RX_AILERON_CHAN);
                float elevator = getRXChannel(RX_ELEVATOR_CHAN);
                float rudder = getRXChannel(RX_RUDDER_CHAN);
                //UARTprintf("%d\t%d\t%d\n", (int16_t)aileron, (int16_t)elevator, (int16_t)rudder);

#if USE_MAG
                if (angleMode && getRXChannel(RX_AUX1_CHAN) > 50 && fabsf(rudder) < 5) { // Only use heading hold if user is not applying rudder and in angle mode
                    static const uint8_t headMaxAngle = 25;
                    if (fmaxf(fabsf(angle.axis.roll), fabsf(angle.axis.pitch)) < headMaxAngle) { // Check that we are not tilted too much
                        float dif = angle.axis.yaw - magHold;
                        if (dif < -180.0f) // Convert range back to [-180:180]
                            dif += 360.0f;
                        if (dif > 180.0f)
                            dif -= 360.0f;
                        rudder -= dif * cfg.headKp;
                        GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED
                    } else
                        GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED
                } else {
                    GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED
                    magHold = angle.axis.yaw;
                }
#endif

                float setPointRoll, setPointPitch; // Roll and pitch control can both be gyro or accelerometer based
                const float setPointYaw = rudder * cfg.stickScalingYaw; // Yaw is always gyro controlled
                if (angleMode) { // Angle mode
                    setPointRoll = constrain(aileron, -cfg.maxAngleInclination, cfg.maxAngleInclination) - angle.axis.roll;
                    setPointPitch = constrain(elevator, -cfg.maxAngleInclination, cfg.maxAngleInclination) - angle.axis.pitch;
                    setPointRoll *= cfg.angleKp;
                    setPointPitch *= cfg.angleKp;
                } else { // Acro mode
                    setPointRoll = aileron * cfg.stickScalingRollPitch;
                    setPointPitch = elevator * cfg.stickScalingRollPitch;
                }

                /*UARTprintf("%d\t%d\n", (int16_t)setPointRoll, (int16_t)setPointPitch);
                UARTFlushTx(false);*/

                float rollOut = updatePID(&pidRoll, setPointRoll, mpu6500.gyroRate.axis.roll, dt);
                float pitchOut = updatePID(&pidPitch, setPointPitch, mpu6500.gyroRate.axis.pitch, dt);
                float yawOut = updatePID(&pidYaw, setPointYaw, -mpu6500.gyroRate.axis.yaw, dt); // Gyro rate is inverted, so it works well with RC yaw control input

                float motors[4]; // Motor 0 is bottom right, motor 1 is top right, motor 2 is bottom left and motor 3 is top left
                float throttle = getRXChannel(RX_THROTTLE_CHAN);

#if USE_SONAR
                if (angleMode && getRXChannel(RX_AUX2_CHAN) > 0) { // Altitude hold
                    const float altHoldSetPoint = 150; // 1.5m
#if USE_BARO
                    int16_t distance = getSonarDistance(&bmp180);
#else
                    int16_t distance = getSonarDistance();
#endif
                    if (distance < 0) // TODO: Use barometer if we get close to 3m instead
                        distance = 300;
                    float altHoldOut = updatePID(&pidAltHold, altHoldSetPoint, distance, dt);
                    if (fabsf(altHoldOut) > 10) {
                        altHoldOut += altHoldOut > 0 ? -10 : 10; // Subtract deadband
                        throttle = constrain(throttle + altHoldOut, -100.0f, 100.0f);
                    }
                }
#endif

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

                // Find the maximum motor output
                float maxMotor = motors[0];
                for (uint8_t i = 1; i < 4; i++) {
                    // If one motor is above the maxthrottle threshold, we reduce the value
                    // of all motors by the amount of overshoot. That way, only one motor
                    // is at max and the relative power of each motor is preserved
                    if (motors[i] > maxMotor)
                        maxMotor = motors[i];
                }

                for (uint8_t i = 0; i < 4; i++) {
                    if (maxMotor > MAX_MOTOR_OUT)
                        motors[i] -= maxMotor - MAX_MOTOR_OUT; // This is a way to still have good gyro corrections if at least one motor reaches its max
                }

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
                resetPIDTerms();
#if USE_MAG
                magHold = angle.axis.yaw;
#endif
            }
        }

#if USE_SONAR
        triggerSonar(); // Trigger sonar
#endif
    }
}

// TODO:
    // Use sonar distance for something usefull - see: https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/altitudehold.c
        // https://github.com/cleanflight/cleanflight/blob/master/src/main/sensors/sonar.c#L90-L99
    // Android App
        // Self level angle trim
        // Calibrate magnetometer
        // Set magnetic declination
        // Set acc_lpf_factor, gyro_cmpf_factor and gyro_cmpfm_factor + add explanation
        // headMaxAngle
    // Add disarm timer
    // Check that both buttons are held in while calibrating ESCs
    // Magnetometer
        // Dynamically adjust gain when calibrating if limit is reached
        // Take average of several values for gain
        // Board orientation
    // Simplify the way PID values are set via Bluetooth
        // Can just set a point to the struct
    // Do not hardcode altitude hold value
