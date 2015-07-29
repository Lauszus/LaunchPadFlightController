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
#include <math.h>

#include "IMU.h"
#include "MPU6500.h"

#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

static float calculateHeading(angle_t *angle, sensor_t *mag);

// Accelerometer readings can be in any scale, but gyro rate needs to be in deg/s
// Make sure that roll increases when tilting quadcopter to the right, pitch increases
// when pitching quadcopter downward and yaw increases when rotating quadcopter clockwise.
void getAngles(mpu6500_t *mpu6500, sensor_t *mag, angle_t *angle, float dt) {
    static const float acc_lpf_factor = 4.0f;
    static const float gyro_cmpf_factor = 600.0f;
#ifdef DEBUG
    const float invGyroComplimentaryFilterFactor = (1.0f / (gyro_cmpf_factor + 1.0f));
#else
    static const float invGyroComplimentaryFilterFactor = (1.0f / (gyro_cmpf_factor + 1.0f));
#endif

    static sensor_t accLPF; // Accelerometer values after low pass filter
    float accMagSquared = 0; // Accelerometer magneturde squared
    sensor_t gyro; // Gyro readings in rad/s

    for (uint8_t axis = 0; axis < 3; axis++) {
        gyro.data[axis] = mpu6500->gyroRate.data[axis] * DEG_TO_RAD; // Convert from deg/s to rad/s
        accLPF.data[axis] = accLPF.data[axis] * (1.0f - (1.0f / acc_lpf_factor)) + (float)mpu6500->acc.data[axis] * (1.0f / acc_lpf_factor); // Apply low pass filter
        accMagSquared += accLPF.data[axis] * accLPF.data[axis]; // Update magnitude
    }

    angle_t deltaAngle = { .data = { gyro.axis.X * dt, gyro.axis.Y * dt, gyro.axis.Z * dt } };
    rotateV(&mpu6500->accBodyFrame, &deltaAngle); // Rotate body frame according to delta angle given by gyro reading

    accMagSquared /= mpu6500->accScaleFactor * mpu6500->accScaleFactor; // Convert readings to g's
    if (0.72f < accMagSquared && accMagSquared < 1.32f) { // Check if < 0.85G or > 1.15G, if so we just skip new accelerometer readings
        for (uint8_t axis = 0; axis < 3; axis++)
            mpu6500->accBodyFrame.data[axis] = (mpu6500->accBodyFrame.data[axis] * gyro_cmpf_factor + accLPF.data[axis]) * invGyroComplimentaryFilterFactor; // Complimentary filter accelerometer gyro readings
    }

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -p to p (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#if 0 // Set to 0 to restrict roll to +-90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
    // Eq. 25 and 26
    angle->axis.roll = atan2f(mpu6500->accBodyFrame.axis.Y, mpu6500->accBodyFrame.axis.Z);
    angle->axis.pitch  = atan2f(-mpu6500->accBodyFrame.axis.X, sqrtf(mpu6500->accBodyFrame.axis.Y * mpu6500->accBodyFrame.axis.Y + mpu6500->accBodyFrame.axis.Z * mpu6500->accBodyFrame.axis.Z)); // Use atan2 here anyway, to prevent division by 0
#else
    // Eq. 28 and 29
    angle->axis.roll = atan2f(mpu6500->accBodyFrame.axis.Y, sqrtf(mpu6500->accBodyFrame.axis.X * mpu6500->accBodyFrame.axis.X + mpu6500->accBodyFrame.axis.Z * mpu6500->accBodyFrame.axis.Z)); // Use atan2 here anyway, to prevent division by 0
    angle->axis.pitch  = atan2f(-mpu6500->accBodyFrame.axis.X, mpu6500->accBodyFrame.axis.Z);
#endif

#if USE_MAG
    static const float gyro_cmpfm_factor = 250.0f;
#ifdef DEBUG
    const float invGyroComplimentaryFilter_M_Factor = (1.0f / (gyro_cmpfm_factor + 1.0f));
#else
    static const float invGyroComplimentaryFilter_M_Factor = (1.0f / (gyro_cmpfm_factor + 1.0f));
#endif

    static sensor_t magBodyFrame; // Magnitude of the earth magnetic field in the body frame

    rotateV(&magBodyFrame, &deltaAngle); // Rotate body frame according to delta angle given by the gyro reading
    for (uint8_t axis = 0; axis < 3; axis++) // TODO: Should this only be done when there is actual new magnetometer data?
        magBodyFrame.data[axis] = (magBodyFrame.data[axis] * gyro_cmpfm_factor + mag->data[axis]) * invGyroComplimentaryFilter_M_Factor; // Use complimentary filter on magnetometer values
    angle->axis.yaw = calculateHeading(angle, &magBodyFrame); // Get heading in degrees
#else
    rotateV(mag, &deltaAngle); // Rotate body frame according to delta angle given by the gyro reading
    angle->axis.yaw = calculateHeading(angle, mag); // Get heading in degrees
#endif

    // Convert readings to degrees
    angle->axis.roll *= RAD_TO_DEG;
    angle->axis.pitch *= RAD_TO_DEG;

#if 0 && UART_DEBUG
    static angle_t gyroAngle;
    for (uint8_t axis = 0; axis < 3; axis++)
        gyroAngle.data[axis] += mpu6500->gyroRate.data[axis] * dt; // Gyro angle is only used for debugging

    UARTprintf("%d\t%d\t", (int16_t)gyroAngle.axis.roll, (int16_t)angle->axis.roll);
    UARTprintf("%d\t%d\t", (int16_t)gyroAngle.axis.pitch, (int16_t)angle->axis.pitch);
    UARTprintf("%d\t%d\n", (int16_t)gyroAngle.axis.yaw, (int16_t)angle->axis.yaw);
    UARTFlushTx(false);
#endif
}

// See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf eq. 22
// Note heading is inverted, so it increases when rotating clockwise. This is done so it works well with the RC yaw control input
static float calculateHeading(angle_t *angle, sensor_t *mag) {
#if USE_MAG
    #ifndef DEBUG
        // Calculate magnetic declination
        static const int16_t magDeclinationFromConfig = -317; // Get your local magnetic declination here: http://magnetic-declination.com (Mine was +3 deg 17')
        static const int16_t deg = magDeclinationFromConfig / 100;
        static const int16_t min = magDeclinationFromConfig % 100;
        static const float magneticDeclination = (deg + ((float)min * (1.0f / 60.0f)));
    #else
        static const float magneticDeclination = -3.28f;
    #endif
#endif

    float cosx = cosf(angle->axis.roll);
    float sinx = sinf(angle->axis.roll);
    float cosy = cosf(angle->axis.pitch);
    float siny = sinf(angle->axis.pitch);

    float Bfy = mag->axis.Z * sinx - mag->axis.Y * cosx;
    float Bfx = mag->axis.X * cosy + mag->axis.Y * siny * sinx + mag->axis.Z * siny * cosx;
#if USE_MAG
    float heading = -(atan2f(Bfy, Bfx) * RAD_TO_DEG + magneticDeclination); // Return heading
#else
    float heading = -(atan2f(Bfy, Bfx) * RAD_TO_DEG); // Return heading
#endif

    if (heading < 0) // Convert heading range to 0-360
        heading += 360;
    return heading;
}

// Rotate accelerometer coordinate axis by a delta angle from gyroscope
// See: http://mathworld.wolfram.com/RotationMatrix.html,
// http://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Euler_angles_.28x-y-z_extrinsic.29_.E2.86.92_Rotation_matrix and
// https://engineering.purdue.edu/~bethel/rot2.pdf
void rotateV(sensor_t *v, angle_t *angle) {
    sensor_t v_tmp = *v;

    float cosx = cosf(angle->axis.roll);
    float sinx = sinf(angle->axis.roll);
    float cosy = cosf(angle->axis.pitch);
    float siny = sinf(angle->axis.pitch);
    float cosz = cosf(angle->axis.yaw);
    float sinz = sinf(angle->axis.yaw);

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

    v->axis.X = v_tmp.axis.X * mat[0][0] + v_tmp.axis.Y * mat[1][0] + v_tmp.axis.Z * mat[2][0];
    v->axis.Y = v_tmp.axis.X * mat[0][1] + v_tmp.axis.Y * mat[1][1] + v_tmp.axis.Z * mat[2][1];
    v->axis.Z = v_tmp.axis.X * mat[0][2] + v_tmp.axis.Y * mat[1][2] + v_tmp.axis.Z * mat[2][2];
}
