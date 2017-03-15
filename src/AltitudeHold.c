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

#if USE_SONAR || USE_BARO || USE_LIDAR_LITE

#include "AltitudeHold.h"
#include "BMP180.h"
#include "Buzzer.h"
#include "IMU.h"
#include "LidarLiteV3.h"
#include "MPU6500.h"
#include "StepResponse.h"
#include "PID.h"
#include "PPM.h"
#include "RX.h"
#include "Sonar.h"
#include "uartstdio1.h" // Add "UART_BUFFERED1" to preprocessor - it uses a modified version of uartstdio, so it can be used with another UART interface

#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#if USE_SONAR
#define SONAR_MIN_DIST 50    // Limit minimum value to 5 cm
#define SONAR_MAX_DIST 1500  // Limit maximum altitude to 1.5 m which is in practice the limit of the sonar
#endif

#if USE_LIDAR_LITE
#define LIDAR_MIN_DIST 1000  // Limit minimum value to 1 m
#define LIDAR_MAX_DIST 40000 // Limit maximum altitude to 40 m
#endif

#if USE_BARO
static bmp180_t bmp180; // Barometer readings
static float altitudeSetPoint;
#endif

static bool altHoldActive;

void initAltitudeHold(void) {
#if USE_SONAR
    initSonar();
#endif
#if USE_LIDAR_LITE
    initLidarLite();
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
void getAltitude(angle_t *angle, mpu6500_t __attribute__((unused)) *mpu6500, altitude_t *altitude, uint32_t __attribute__((unused)) now, float __attribute__((unused)) dt) {
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

#if USE_LIDAR_LITE
    if (triggerLidarLite()) // Trigger lidar lite
        altitude->lidarLiteDistance = getLidarLiteDistance(angle);
#endif

#if USE_SONAR && USE_LIDAR_LITE
    if (altitude->sonarDistance >= 0 && altitude->sonarDistance <= LIDAR_MIN_DIST) // Use only sonar measurement when below the minimum distance of the LIDAR-Lite v3
        altitude->distance = altitude->sonarDistance;
    else if (altitude->lidarLiteDistance >= SONAR_MAX_DIST) // Use LIDAR-Lite v3 measurement if we exceed the maximum limit of the sonar
        altitude->distance = altitude->lidarLiteDistance;
    else {
        if (altitude->sonarDistance < 0) // Invalid result
            altitude->distance = altitude->lidarLiteDistance; // So use LIDAR-Lite v3 only, this might actually set the distance to -1 if the LIDAR is also returning an invalid result, but this is handled in the altitude hold code
        else if (altitude->lidarLiteDistance < 0) // Invalid result
            altitude->distance = altitude->sonarDistance; // Same the other way around
        else { // Combine the measurements from both sensors
            const float dist_coef = (float)(constrain(altitude->distance, LIDAR_MIN_DIST, SONAR_MAX_DIST) - LIDAR_MIN_DIST) / (float)(SONAR_MAX_DIST - LIDAR_MIN_DIST); // Constrain in case distance was not previously set and then calculate value in the range [0,1]
            altitude->distance = (float)altitude->sonarDistance * (1.0f - dist_coef) + (float)altitude->lidarLiteDistance * dist_coef; // Make smooth transaction between the two sensors
        }
    }
#elif USE_SONAR
    altitude->distance = altitude->sonarDistance; // Only sonar distance is available
#elif USE_LIDAR_LITE
    altitude->distance = altitude->lidarLiteDistance; // Only LIDAR-Lite v3 is available
#endif // USE_SONAR && USE_LIDAR_LITE
#if 0
    UARTprintf("Distance: %d.%02u\n", (int32_t)altitude->distance, (uint32_t)(abs(altitude->distance * 100.0f) % 100));
    UARTFlushTx(false);
#endif

#if USE_BARO
    if (getBMP180Data(&bmp180)) {
#if 0
        float height = bmp180.absoluteAltitude - bmp180.groundAltitude;
        UARTprintf1("%d,%d,%d.%02u\n", bmp180.pressure, bmp180.temperature, (int32_t)height, (uint32_t)(abs(height * 100.0f) % 100));
        UARTFlushTx1(false);
#endif
    }

    /* Estimate altitude and velocity using barometer */
    static float baro_noise_lpf = 0.95f; // TODO: Set via app
    static float baroAltitude;

    float lastBaroAltitude = baroAltitude;
    baroAltitude = baro_noise_lpf * baroAltitude + (1.0f - baro_noise_lpf) * (bmp180.absoluteAltitude - bmp180.groundAltitude); // LPF to reduce baro noise

#if USE_SONAR && 1
    // TODO: Add smooth transaction between sonar and barometer
    static float altitudeBaroOffset;
    if (altitude->sonarDistance >= 0 && altitude->sonarDistance < /*2000*/100) {
        float sonarHeight = (float)altitude->sonarDistance / 10.0f; // Convert sonar distance to cm
        altitudeBaroOffset = baroAltitude - sonarHeight; // Set altitude offset
        baroAltitude = sonarHeight; // Set altitude estimate equal to sonar height
    } else
        baroAltitude -= altitudeBaroOffset; // Subtract offset from altitude estimate
#endif

    float baroVelocity = (baroAltitude - lastBaroAltitude) / dt; // Estimate baro velocity

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

    /* Estimate altitude and velocity using acceleration */
    // Subtract subtract 1g datasheet value, so it reads 0g when it is flat and invert z-axis so it is pointing upward
    float accelerationZ = -(accInertialFrame.axis.Z - mpu6500->accScaleFactor);
    // Convert into g's, then into m/s^2 and finally into cm/s^2
    static const float gravitationalAcceleration = 9.80665f * 100.0f; // See: https://en.wikipedia.org/wiki/Gravitational_acceleration
    altitude->acceleration = accelerationZ / mpu6500->accScaleFactor * gravitationalAcceleration;

    float accDt = altitude->acceleration * dt; // Limit number of multiplications
    float accVelocity = altitude->velocity + accDt; // Estimate velocity using acceleration
    float accAltitude = altitude->altitude + altitude->velocity * dt + 0.5f * accDt * dt; // Estimate altitude using acceleration

    /* Estimate altitude and velocity using complimentary filter on barometer and acceleration estimates */
    static const float velocity_cf = 0.985f; // TODO: Set in Android app
    altitude->velocity = velocity_cf * accVelocity + (1 - velocity_cf) * baroVelocity; // Estimate velocity using complimentary filter

    static const float altitude_cf = 0.965f; // TODO: Set in Android app
    altitude->altitude = altitude_cf * accAltitude + (1 - altitude_cf) * baroAltitude; // Estimate altitude using complimentary filter

    static const float altitude_lpf = 0.995f; // TODO: Set in Android app
    altitude->altitudeLpf = altitude_lpf * altitude->altitudeLpf + (1.0f - altitude_lpf) * altitude->altitude; // Low-pass filter altitude estimate

    //UARTprintf1("%d\t%d\n", (int32_t)baroAltitude, (int32_t)baroVelocity);
    //UARTprintf1("%d\t%d\t%d\n", (int32_t)accAltitude, (int32_t)accVelocity, (int32_t)altitude->acceleration);
    //UARTprintf1("%d\t%d\t%d\t%d\n", (int32_t)altitude->altitudeLpf, (int32_t)altitude->altitude, (int32_t)altitude->velocity, (int32_t)altitude->acceleration);
    //UARTFlushTx1(false);

#endif // USE_BARO
}

// TODO: Handle if distance is -1 in a better way
float updateAltitudeHold(float aux, altitude_t *altitude, float throttle, uint32_t __attribute__((unused)) now, float dt) {
    static const float MIN_MOTOR_OFFSET = (MAX_MOTOR_OUT - MIN_MOTOR_OUT) * 0.05f; // Add 5% to minimum, so the motors are never completely shut off
    static float altHoldInitialThrottle = -30.0f; // Throttle when altitude hold was activated

#if USE_SONAR || USE_LIDAR_LITE
    static const float throttle_noise_lpf = 1000.0f; // TODO: Set via app
    static float altHoldThrottle; // Low pass filtered throttle input
    static int16_t altHoldSetPoint; // Altitude hold set point

    if (aux < 60 && altitude->distance >= 0) { // Make sure the distance is valid
        if (!altHoldActive) { // We just went from deactivated to active
            altHoldActive = true;
            resetPIDAltHold();
            altHoldThrottle = throttle; // Set low pass filtered throttle value

            if (altHoldInitialThrottle < CHANNEL_MIN_CHECK) { // If throttle is very low, just set an initial value, so it still works
                // TODO: Don't hardcode these values
                altHoldSetPoint = 1000; // Set to 1m
                altHoldInitialThrottle = -30.0f; // Set the throttle value to where is approximately hovers
            } else {
#if USE_SONAR && USE_LIDAR_LITE
                altHoldSetPoint = constrain(altitude->distance, SONAR_MIN_DIST, LIDAR_MAX_DIST); // Constrain set point to the min and max allowed
#elif USE_SONAR
                altHoldSetPoint = constrain(altitude->distance, SONAR_MIN_DIST, SONAR_MAX_DIST); // Constrain set point to the min and max allowed
#elif USE_LIDAR_LITE
                altHoldSetPoint = constrain(altitude->distance, LIDAR_MIN_DIST, LIDAR_MAX_DIST); // Constrain set point to the min and max allowed
#endif // USE_SONAR && USE_LIDAR_LITE
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

        altHoldThrottle = altHoldThrottle * (1.0f - (1.0f / throttle_noise_lpf)) + throttle * (1.0f / throttle_noise_lpf); // LPF throttle input

        float setPoint;
#if !STEP_ALTITUDE_HOLD
#if USE_SONAR && USE_LIDAR_LITE
        if (altHoldThrottle < altHoldInitialThrottle)
            setPoint = mapf(altHoldThrottle, MIN_MOTOR_OUT, altHoldInitialThrottle, SONAR_MIN_DIST, altHoldSetPoint);
        else
            setPoint = mapf(altHoldThrottle, altHoldInitialThrottle, MAX_MOTOR_OUT, altHoldSetPoint, LIDAR_MAX_DIST);
#elif USE_SONAR
        if (altHoldThrottle < altHoldInitialThrottle)
            setPoint = mapf(altHoldThrottle, MIN_MOTOR_OUT, altHoldInitialThrottle, SONAR_MIN_DIST, altHoldSetPoint);
        else
            setPoint = mapf(altHoldThrottle, altHoldInitialThrottle, MAX_MOTOR_OUT, altHoldSetPoint, SONAR_MAX_DIST);
#elif USE_LIDAR_LITE
        if (altHoldThrottle < altHoldInitialThrottle)
            setPoint = mapf(altHoldThrottle, MIN_MOTOR_OUT, altHoldInitialThrottle, LIDAR_MIN_DIST, altHoldSetPoint);
        else
            setPoint = mapf(altHoldThrottle, altHoldInitialThrottle, MAX_MOTOR_OUT, altHoldSetPoint, LIDAR_MAX_DIST);
#endif // USE_SONAR && USE_LIDAR_LITE
#else
        // This code is only used when logging is used, so it is easy to map between distance and throttle values
        setPoint = mapf(altHoldThrottle, MIN_MOTOR_OUT, MAX_MOTOR_OUT, SONAR_MIN_DIST, SONAR_MAX_DIST);
#endif // !STEP_ALTITUDE_HOLD

        float altHoldOut = updatePID(&pidSonarAltHold, setPoint, altitude->distance, dt);
        throttle = constrain(altHoldInitialThrottle + altHoldOut, MIN_MOTOR_OUT + MIN_MOTOR_OFFSET, MAX_MOTOR_OUT); // Throttle value is set to throttle when altitude hold were first activated plus output from PID controller
        /*UARTprintf("%u %d %d %d - %d %d %d %d\n", altHoldActive, (int32_t)altHoldThrottle, (int32_t)altHoldInitialThrottle, altHoldSetPoint,     (int32_t)setPoint, altitude->distance, (int32_t)altHoldOut, (int32_t)throttle);
        UARTFlushTx(false);*/
    }
#if USE_BARO
    else
#endif // USE_BARO
#endif // USE_SONAR || USE_LIDAR_LITE

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

void resetAltitudeHold(altitude_t __attribute__((unused)) *altitude) {
    altHoldActive = false;
#if USE_BARO
    altitudeSetPoint = altitude->altitudeLpf;
#endif // USE_BARO
}

#endif // USE_SONAR || USE_BARO || USE_LIDAR_LITE
