/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

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

#ifndef __kalman_h__
#define __kalman_h__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
} kalman_t;

void KalmanInit(kalman_t *kalman);

float getAngle(kalman_t *kalman, float newAngle, float newRate, float dt);
float getRate(kalman_t *kalman);
float getQangle(kalman_t *kalman);
float getQbias(kalman_t *kalman);
float getRmeasure(kalman_t *kalman);

void setAngle(kalman_t *kalman, float angle);
void setQangle(kalman_t *kalman, float Q_angle);
void setQbias(kalman_t *kalman, float Q_bias);
void setRmeasure(kalman_t *kalman, float R_measure);

#ifdef __cplusplus
}
#endif

#endif
