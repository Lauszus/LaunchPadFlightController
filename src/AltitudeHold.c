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

// Inspired by: https://github.com/cleanflight/cleanflight/blob/master/src/main/sensors/barometer.c and https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/altitudehold.c
// Note that I reused the filtered value from the attitude estimation instead of filtering the LPF accelerometer data like Cleanflight does

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#if USE_SONAR || USE_BARO

#include "AltitudeHold.h"
#include "BMP180.h"
#include "Buzzer.h"
#include "IMU.h"
#include "MPU6500.h"
#include "StepResponse.h"
#include "PID.h"
#include "PPM.h"
#include "RX.h"
#include "Sonar.h"
#include "Types.h"
#include "uartstdio1.h" // Add "UART_BUFFERED1" to preprocessor - it uses a modified version of uartstdio, so it can be used with another UART interface

#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#define SONAR_MIN_DIST 50   // Limit minimum value to 5cm
#define SONAR_MAX_DIST 1500 // Limit maximum altitude to 1.5m which is in practice the limit of the sonar

#if USE_BARO
static bmp180_t bmp180; // Barometer readings
static float altitudeSetPoint;
#endif

static bool altHoldActive;

void initAltitudeHold(void) {
#if USE_SONAR
    initSonar();
#endif
#if USE_BARO
    initBMP180(&bmp180);
#endif

#if UART_DEBUG && USE_BARO
    UARTprintf("Barometer values: %d\t%d\t%d\n", bmp180.pressure, bmp180.temperature, (int32_t)bmp180.groundAltitude);
#endif
}

// TODO: LPF mpu6500->accBodyFrame.axis.Z
// TODO: Maybe the altitude should only be run when new barometer values have been read and then just use a moving average on the acceleration data
// TODO: Reset acceleration estimate when unarmed, as we can assumed that it is at rest
void getAltitude(angle_t *angle, mpu6500_t *mpu6500, altitude_t *altitude, uint32_t __attribute__((unused)) now, float dt) {
#if USE_SONAR
    if (triggerSonar()) { // Trigger sonar
#if USE_BARO
        altitude->sonarDistance = getSonarDistance(angle, bmp180.temperature);
#else
        altitude->sonarDistance = getSonarDistance(angle);
#endif

#if 0
        static uint32_t counter;
        UARTprintf1("%u,%u,%d.%02u,%d.%02u\n",
                                              ++counter,
                                              now,
                                              (int16_t)altitude->altitudeLpf, (uint16_t)(abs(altitude->altitudeLpf * 100.0f) % 100),
                                              (int16_t)altitude->altitude, (uint16_t)(abs(altitude->altitude * 100.0f) % 100));
        UARTFlushTx1(false);
#endif
    }
#endif

#if USE_BARO
    if (getBMP180Data(&bmp180)) {
#if 0
        float height = bmp180.absoluteAltitude - bmp180.groundAltitude;
        UARTprintf1("%d,%d,%d.%02u\n", bmp180.pressure, bmp180.temperature, (int32_t)height, (uint32_t)(abs(height * 100.0f) % 100));
        UARTFlushTx1(false);
#endif
    }

    /* Rotate body frame into inertial frame */
    angle_t rotAngle = {
            .axis = {
                    .roll = -angle->axis.roll * DEG_TO_RAD,
                    .pitch = -angle->axis.pitch * DEG_TO_RAD,
                    .yaw = -angle->axis.yaw * DEG_TO_RAD,
            }
    };
    sensor_t accInertialFrame = mpu6500->accBodyFrame;
    rotateV(&accInertialFrame, &rotAngle);
    float accelerationZ = -accInertialFrame.axis.Z; // Since the z-axis is pointing downward we invert it

    /* Estimate altitude and velocity using barometer */
    // Low-pass filter altitude estimate - see: https://en.wikipedia.org/wiki/Exponential_smoothing
    static const float baro_lpf_Fc = 8.38f; // Cutoff frequency in Hz - TODO: Set in Android app
    static const float baro_lpf_tau = 1.0f/(2.0f*M_PIf*baro_lpf_Fc); // tau = 1.0f/(2.0f*Pi*Fc) = .019
    const float baro_alpha = dt/(baro_lpf_tau + dt); // alpha = dt/(tau + dt) = .05

    static float baroAltitude;
    float lastBaroAltitude = baroAltitude;
    baroAltitude = baroAltitude + baro_alpha*((bmp180.absoluteAltitude - bmp180.groundAltitude) - baroAltitude); // y(n) = y(n-1) + alpha*(u(n) - y(n-1))

#if USE_SONAR && 1
    // TODO: Add smooth transaction between sonar and barometer
    static float altitudeBaroOffset;
    if (altitude->sonarDistance > 0 && altitude->sonarDistance < /*2000*/100) {
        float sonarHeight = (float)altitude->sonarDistance / 10.0f; // Convert sonar distance to cm
        altitudeBaroOffset = baroAltitude - sonarHeight; // Set altitude offset
        baroAltitude = sonarHeight; // Set altitude estimate equal to sonar height
    } else
        baroAltitude -= altitudeBaroOffset; // Subtract offset from altitude estimate
#endif

    float baroVelocity = (baroAltitude - lastBaroAltitude) / dt; // Estimate baro velocity

    /* Estimate altitude and velocity using acceleration */
    // Fist subtract 1g datasheet value, so it is reading 0g when it is flat, then the value is actually converted into g's, then into m/s^2 and finally into cm/s^2
    static const float gravitationalAcceleration = 9.80665f; // See: https://en.wikipedia.org/wiki/Gravitational_acceleration
    altitude->acceleration = (float)(accelerationZ - mpu6500->accScaleFactor) / mpu6500->accScaleFactor * gravitationalAcceleration * 100.0f;

    float accDt = altitude->acceleration * dt; // Limit number of multiplications
    float accVelocity = altitude->velocity + accDt; // Estimate velocity using acceleration
    float accAltitude = altitude->altitude + altitude->velocity * dt + 0.5f * accDt * dt; // Estimate altitude using acceleration

    /* Estimate altitude and velocity using complimentary filter on barometer and acceleration estimates */
    static const float velocity_cf = 0.985f; // TODO: Set in Android app
    altitude->velocity = velocity_cf * accVelocity + (1 - velocity_cf) * baroVelocity; // Estimate velocity using complimentary filter

    static const float altitude_cf = 0.965f; // TODO: Set in Android app
    altitude->altitude = altitude_cf * accAltitude + (1 - altitude_cf) * baroAltitude; // Estimate altitude using complimentary filter

    // Low-pass filter altitude estimate - see: https://en.wikipedia.org/wiki/Exponential_smoothing
    static const float altitude_lpf_Fc = .80f; // Cutoff frequency in Hz - TODO: Set in Android app
    static const float altitude_lpf_tau = 1.0f/(2.0f*M_PIf*altitude_lpf_Fc); // tau = 1.0f/(2.0f*Pi*Fc) = .199
    const float altitude_alpha = dt/(altitude_lpf_tau + dt); // alpha = dt/(tau + dt) = .005
    altitude->altitudeLpf = altitude->altitudeLpf + altitude_alpha*(altitude->altitude - altitude->altitudeLpf); // y(n) = y(n-1) + alpha*(u(n) - y(n-1))

    //UARTprintf1("%d\t%d\n", (int32_t)baroAltitude, (int32_t)baroVelocity);
    //UARTprintf1("%d\t%d\t%d\n", (int32_t)accAltitude, (int32_t)accVelocity, (int32_t)altitude->acceleration);
    //UARTprintf1("%d\t%d\t%d\t%d\n", (int32_t)altitude->altitudeLpf, (int32_t)altitude->altitude, (int32_t)altitude->velocity, (int32_t)altitude->acceleration);
    //UARTFlushTx1(false);

#endif // USE_BARO
}

float updateAltitudeHold(float aux, altitude_t *altitude, float throttle, uint32_t __attribute__((unused)) now, float dt) {
    static const float MIN_MOTOR_OFFSET = (MAX_MOTOR_OUT - MIN_MOTOR_OUT) * 0.05f; // Add 5% to minimum, so the motors are never completely shut off
    static float altHoldInitialThrottle = -30.0f; // Throttle when altitude hold was activated

#if USE_SONAR
    static const float throttle_lpf_Fc = .158995947f; // Cutoff frequency in Hz - TODO: Set in Android app
    static const float throttle_lpf_tau = 1.0f/(2.0f*M_PIf*throttle_lpf_Fc); // tau = 1.0f/(2.0f*Pi*Fc) = 1.001
    const float throttle_alpha = dt/(throttle_lpf_tau + dt); // alpha = dt/(tau + dt) = .001

    static float altHoldThrottle; // Low pass filtered throttle input
    static int16_t altHoldSetPoint; // Altitude hold set point

    if (aux < 60 && altitude->sonarDistance >= 0/* && altitude->sonarDistance > 2000*/) { // Make sure the distance is valid
        if (!altHoldActive) { // We just went from deactivated to active
            altHoldActive = true;
            resetPIDAltHold();
            altHoldThrottle = throttle; // Set low pass filtered throttle value

            if (altHoldInitialThrottle < CHANNEL_MIN_CHECK) { // If throttle is very low, just set an initial value, so it still works
                // TODO: Don't hardcode these values
                altHoldSetPoint = 1000; // Set to 1m
                altHoldInitialThrottle = -30.0f; // Set the throttle value to where is approximately hovers
            } else {
                altHoldSetPoint = constrain(altitude->sonarDistance, SONAR_MIN_DIST, SONAR_MAX_DIST); // Constrain set point to the min and max allowed
                altHoldInitialThrottle = throttle; // Save current throttle
            }
        }

#if STEP_ALTITUDE_HOLD
        (void)altHoldSetPoint; // Suppress warning
        const float input = mapf(altitude->sonarDistance, SONAR_MIN_DIST, SONAR_MAX_DIST, MIN_MOTOR_OUT, MAX_MOTOR_OUT);
        const float step1 = mapf(500, SONAR_MIN_DIST, SONAR_MAX_DIST, MIN_MOTOR_OUT, MAX_MOTOR_OUT); // Start at 50cm
        const float step2 = mapf(1000, SONAR_MIN_DIST, SONAR_MAX_DIST, MIN_MOTOR_OUT, MAX_MOTOR_OUT); // Go to 1m
        static const uint32_t interval = 15e6; // 15 seconds between steps
        throttle = stepResponse(getRXChannel(RX_AUX2_CHAN) > 90, throttle, input, step1, step2, interval, now);
#endif

        // Apply exponential smoothing: https://en.wikipedia.org/wiki/Exponential_smoothing
        altHoldThrottle = altHoldThrottle + throttle_alpha*(throttle - altHoldThrottle); // y(n) = y(n-1) + alpha*(u(n) - y(n-1))

        float setPoint;
#if !STEP_ALTITUDE_HOLD
        if (altHoldThrottle < altHoldInitialThrottle)
            setPoint = mapf(altHoldThrottle, MIN_MOTOR_OUT, altHoldInitialThrottle, SONAR_MIN_DIST, altHoldSetPoint);
        else
            setPoint = mapf(altHoldThrottle, altHoldInitialThrottle, MAX_MOTOR_OUT, altHoldSetPoint, SONAR_MAX_DIST);
#else
        // This code is only used when logging is used, so it is easy to map between distance and throttle values
        setPoint = mapf(altHoldThrottle, MIN_MOTOR_OUT, MAX_MOTOR_OUT, SONAR_MIN_DIST, SONAR_MAX_DIST);
#endif

        float altHoldOut = updatePID(&pidSonarAltHold, setPoint, altitude->sonarDistance, dt);
        throttle = constrain(altHoldInitialThrottle + altHoldOut, MIN_MOTOR_OUT + MIN_MOTOR_OFFSET, MAX_MOTOR_OUT); // Throttle value is set to throttle when altitude hold were first activated plus output from PID controller
        /*UARTprintf("%u %d %d %d - %d %d %d %d\n", altHoldActive, (int32_t)altHoldThrottle, (int32_t)altHoldInitialThrottle, altHoldSetPoint,     (int32_t)setPoint, altitude->sonarDistance, (int32_t)altHoldOut, (int32_t)throttle);
        UARTFlushTx(false);*/
    }
#if USE_BARO
    else
#endif // USE_BARO
#endif // USE_SONAR

#if USE_BARO
    if (aux > 60/* && altitude->sonarDistance > 2000*/) {
        if (!altHoldActive) { // We just went from deactivated to active
            altHoldActive = true;
            altHoldInitialThrottle = throttle; // Save current throttle
            resetPIDAltHold();
        }
        float altHoldOut = updatePID(&pidBaroAltHold, altitudeSetPoint * 10.0f, altitude->altitudeLpf * 10.0f, dt); // Multiply by 10 in order to convert it from cm to mm
        throttle = constrain(altHoldInitialThrottle + altHoldOut, MIN_MOTOR_OUT + MIN_MOTOR_OFFSET, MAX_MOTOR_OUT); // Throttle value is set to throttle when altitude hold were first activated plus output from PID controller
        /*UARTprintf1("%d %d %d %d %d\n", (int32_t)altitudeSetPoint, (int32_t)altitude->altitudeLpf, (int32_t)altHoldInitialThrottle, (int32_t)altHoldOut, (int32_t)throttle);
        UARTFlushTx1(false);*/
    }
#endif // USE_BARO

    return throttle;
}

void resetAltitudeHold(altitude_t *altitude) {
    altHoldActive = false;
#if USE_BARO
    altitudeSetPoint = altitude->altitudeLpf;
#endif // USE_BARO
}

#endif // USE_SONAR || USE_BARO
