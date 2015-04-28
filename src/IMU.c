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

// Inspired by: https://github.com/cleanflight/cleanflight/blob/78a4476506c06315d7296a010a2c7ba003146b44/src/main/flight/imu.c

#include <stdint.h>
#include <Math.h>

#include "IMU.h"
#include "MPU6500.h"

#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#define DEG_TO_RAD  0.017453292519943295769236907684886f
#define RAD_TO_DEG  57.295779513082320876798154814105f

static float calculateHeading(angle_t *angle, sensor_t *mag);
static void rotateV(sensor_t *v, sensor_t *gyroRate, float dt);

// Accelerometer readings can be in any scale, but gyro rate needs to be in deg/s
// Make sure that roll increases when tilting quadcopter to the right, pitch increases
// when pitching quadcopter downward and yaw increases when rotating quadcopter clockwise.
void getAngles(mpu6500_t *mpu6500, sensor_t *mag, angle_t *angle, float dt) {
    static const float acc_lpf_factor = 4.0f;
    static const float gyro_cmpf_factor = 600.0f;
    static const float invGyroComplimentaryFilterFactor = (1.0f / (gyro_cmpf_factor + 1.0f));

    static sensor_t accLPF; // Accelerometer values after low pass filter
    static sensor_t accFiltered; // Filtered accelerometer vector
    float accMagSquared = 0; // Accelerometer magneturde squared
    sensor_t gyro; // Gyro readings in rad/s

    for (uint8_t axis = 0; axis < 3; axis++) {
        gyro.data[axis] = mpu6500->gyroRate.data[axis] * DEG_TO_RAD; // Convert from deg/s to rad/s
        accLPF.data[axis] = accLPF.data[axis] * (1.0f - (1.0f / acc_lpf_factor)) + mpu6500->acc.data[axis] * (1.0f / acc_lpf_factor); // Apply low pass filter
        accMagSquared += accLPF.data[axis] * accLPF.data[axis]; // Update magnitude
    }

    rotateV(&accFiltered, &gyro, dt); // Rotate accelerometer vector according to delta angle given by gyro reading

    accMagSquared /= mpu6500->accScaleFactor * mpu6500->accScaleFactor; // Convert readings to g's
    if (0.72f < accMagSquared && accMagSquared < 1.32f) { // Check if < 0.85G or > 1.15G, if so we just skip new accelerometer readings
        for (uint8_t axis = 0; axis < 3; axis++)
            accFiltered.data[axis] = (accFiltered.data[axis] * gyro_cmpf_factor + accLPF.data[axis]) * invGyroComplimentaryFilterFactor; // Complimentary filter accelerometer gyro readings
    }

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -p to p (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#if 0 // Set to 0 to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
    // Eq. 25 and 26
    angle->roll = atan2f(accFiltered.Y, accFiltered.Z);
    angle->pitch  = atan2f(-accFiltered.X, sqrtf(accFiltered.Y * accFiltered.Y + accFiltered.Z * accFiltered.Z)); // Use atan2 here anyway, to prevent division by 0
#else
    // Eq. 28 and 29
    angle->roll = atan2f(accFiltered.Y, sqrtf(accFiltered.X * accFiltered.X + accFiltered.Z * accFiltered.Z)); // Use atan2 here anyway, to prevent division by 0
    angle->pitch  = atan2f(-accFiltered.X, accFiltered.Z);
#endif

#if USE_MAG
    static const float gyro_cmpfm_factor = 250.0f;
    static const float invGyroComplimentaryFilter_M_Factor = (1.0f / (gyro_cmpfm_factor + 1.0f));
    static sensor_t magFiltered; // Filtered magnetometer vector

    rotateV(&magFiltered, &gyro, dt); // Rotate magnetometer vector according to delta angle given by the gyro reading
    for (uint8_t axis = 0; axis < 3; axis++)
        magFiltered.data[axis] = (magFiltered.data[axis] * gyro_cmpfm_factor + mag->data[axis]) * invGyroComplimentaryFilter_M_Factor; // Use complimentary filter on magnetometer values
    angle->yaw = calculateHeading(angle, &magFiltered); // Get heading in degrees
#else
    rotateV(mag, &gyro, dt); // Rotate magnetometer vector according to delta angle given by the gyro reading
    angle->yaw = calculateHeading(angle, mag); // Get heading in degrees
#endif

    // Convert readings to degrees
    angle->roll *= RAD_TO_DEG;
    angle->pitch *= RAD_TO_DEG;

#if 0 && UART_DEBUG
    static angle_t gyroAngle;
    for (uint8_t axis = 0; axis < 3; axis++)
        gyroAngle.data[axis] += mpu6500->gyroRate.data[axis] * dt; // Gyro angle is only used for debugging

    UARTprintf("%d\t%d\t", (int16_t)gyroAngle.roll, (int16_t)angle->roll);
    UARTprintf("%d\t%d\t", (int16_t)gyroAngle.pitch, (int16_t)angle->pitch);
    UARTprintf("%d\t%d\n", (int16_t)gyroAngle.yaw, (int16_t)angle->yaw);
    UARTFlushTx(false);
#endif
}

// See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf eq. 22
// Note heading is inverted, so it increases when rotating clockwise. This is done so it works well with the RC yaw control input
static float calculateHeading(angle_t *angle, sensor_t *mag) {
#if USE_MAG
    // Calculate magnetic declination
    static const int16_t magDeclinationFromConfig = -317; // Get your local magnetic declination here: http://magnetic-declination.com (Mine was +3 deg 17')
    static const int16_t deg = magDeclinationFromConfig / 100;
    static const int16_t min = magDeclinationFromConfig % 100;
    static const float magneticDeclination = (deg + ((float)min * (1.0f / 60.0f)));
#endif

    float cosx = cosf(angle->roll);
    float sinx = sinf(angle->roll);
    float cosy = cosf(angle->pitch);
    float siny = sinf(angle->pitch);

    float Bfy = mag->Z * sinx - mag->Y * cosx;
    float Bfx = mag->X * cosy + mag->Y * siny * sinx + mag->Z * siny * cosx;
#if USE_MAG
    float heading = -(atan2f(Bfy, Bfx) * RAD_TO_DEG + magneticDeclination); // Return heading
#else
    float heading = -(atan2f(Bfy, Bfx) * RAD_TO_DEG); // Return heading
#endif

    if (heading < 0) // Convert heading range to 0-360
        heading += 360;
    return heading;
}

// Rotate accelerometer sensor readings by a delta angle from gyroscope
// See: http://inside.mines.edu/fs_home/gmurray/ArbitraryAxisRotation
static void rotateV(sensor_t *v, sensor_t *gyroRate, float dt) {
    sensor_t v_tmp = *v;
    angle_t deltaAngle = { .data = { gyroRate->X * dt, gyroRate->Y * dt, gyroRate->Z * dt } };

    float cosx = cosf(deltaAngle.roll);
    float sinx = sinf(deltaAngle.roll);
    float cosy = cosf(deltaAngle.pitch);
    float siny = sinf(deltaAngle.pitch);
    float cosz = cosf(deltaAngle.yaw);
    float sinz = sinf(deltaAngle.yaw);

    float coszcosx = cosz * cosx;
    float sinzcosx = sinz * cosx;
    float coszsinx = sinx * cosz;
    float sinzsinx = sinx * sinz;

    float mat[3][3];
    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    v->X = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}
