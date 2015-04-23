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

#include <Math.h>
#include <stdint.h>

#include "IMU.h"
#include "MPU6500.h"

#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#define DEG_TO_RAD  0.017453292519943295769236907684886f
#define RAD_TO_DEG  57.295779513082320876798154814105f

static float imuCalculateHeading(angle_t *angle, sensor_t *mag);
static void rotateV(sensor_t *v, sensor_t *gyroRate, float dt);
static void normalizeV(sensor_t *src, sensor_t *dest);

// Accelerometer readings can be in any scale, but gyro rate needs to be in deg/s
// Make sure that roll increases when tilting quadcopter to the right, pitch increases
// when pitching quadcopter downward and yaw increases when rotating quadcopter clockwise.
void calculateAngles(mpu6500_t *mpu6500, angle_t *angle, float dt) {
    static const float acc_lpf_factor = 4.0f;
    static const float gyro_cmpf_factor = 600.0f;
    static const float invGyroComplimentaryFilterFactor = (1.0f / (gyro_cmpf_factor + 1.0f));
    float accMagSquared = 0; // Accelerometer magneturde squared

    static sensor_t accLPF; // Accelerometer values after low pass filter
    sensor_t gyro; // Gyro readings in rad/s
    static sensor_t acc, mag = { .data = { 1.0f, 0.0f, 0.0f } }; // Accelerometer and magnetometer vectors - note that a magnetometer is currently not connected, so just define a vector with a x-component only

    for (uint8_t axis = 0; axis < 3; axis++) {
        gyro.data[axis] = mpu6500->gyroRate.data[axis] * DEG_TO_RAD; // Convert from deg/s to rad/s
        accLPF.data[axis] = accLPF.data[axis] * (1.0f - (1.0f / acc_lpf_factor)) + mpu6500->acc.data[axis] * (1.0f / acc_lpf_factor); // Apply low pass filter
        accMagSquared += accLPF.data[axis] * accLPF.data[axis] / (MPU6500_ACC_SCALE_FACTOR * MPU6500_ACC_SCALE_FACTOR); // Convert readings to g's
    }

    rotateV(&acc, &gyro, dt); // Rotate accelerometer vector according to delta angle given by gyro reading

    if (0.72f < accMagSquared && accMagSquared < 1.32f) { // Check if < 0.85G or > 1.15G, if so we just skip new accelerometer readings
        for (uint8_t axis = 0; axis < 3; axis++)
            acc.data[axis] = (acc.data[axis] * gyro_cmpf_factor + accLPF.data[axis]) * invGyroComplimentaryFilterFactor; // Complimentary filter accelerometer gyro readings
    }

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -p to p (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#if 0 // Set to 0 to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
    // Eq. 25 and 26
    angle->roll = atan2f(acc.Y, acc.Z);
    angle->pitch  = atan2f(-acc.X, sqrtf(acc.Y * acc.Y + acc.Z * acc.Z)); // Use atan2 here anyway, to prevent division by 0
#else
    // Eq. 28 and 29
    angle->roll = atan2f(acc.Y, sqrtf(acc.X * acc.X + acc.Z * acc.Z)); // Use atan2 here anyway, to prevent division by 0
    angle->pitch  = atan2f(-acc.X, acc.Z);
#endif

    rotateV(&mag, &gyro, dt); // Rotate magnetometer vector according to delta angle given by the gyro reading
    normalizeV(&mag, &mag); // Normalize magnetometer vector
    angle->yaw = imuCalculateHeading(angle, &mag); // Get heading

    // Convert readings to degrees
    angle->roll *= RAD_TO_DEG;
    angle->pitch *= RAD_TO_DEG;
    angle->yaw *= RAD_TO_DEG;

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
static float imuCalculateHeading(angle_t *angle, sensor_t *mag) {
  float Bfy = mag->Z * sinf(angle->roll) - mag->Y * cosf(angle->roll);
  float Bfx = mag->X * cosf(angle->pitch) + mag->Y * sinf(angle->pitch) * sinf(angle->roll) + mag->Z * sinf(angle->pitch) * cosf(angle->roll);
  return -atan2f(Bfy, Bfx); // Return heading
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

// Normalize a vector
static void normalizeV(sensor_t *src, sensor_t *dest) {
  float magnitude = sqrtf(src->X * src->X + src->Y * src->Y + src->Z * src->Z);
  if (magnitude != 0) {
    dest->X = src->X / magnitude;
    dest->Y = src->Y / magnitude;
    dest->Z = src->Z / magnitude;
  }
}
