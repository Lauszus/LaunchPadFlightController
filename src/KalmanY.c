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

// I modifed it, so it can be used with C99 instead of C++

#include "KalmanY.h"

/* Kalman filter variables */
float Q_angleY; // Process noise variance for the accelerometer
float Q_biasY; // Process noise variance for the gyro bias
float R_measureY; // Measurement noise variance - this is actually the variance of the measurement noise

float angleY; // The angle calculated by the Kalman filter - part of the 2x1 state vector
float biasY; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rateY; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

float PY[2][2]; // Error covariance matrix - This is a 2x2 matrix

void KalmanYInit(void) {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angleY = 0.001f;
    Q_biasY = 0.003f;
    R_measureY = 0.03f;

    angleY = 0.0f; // Reset the angle
    biasY = 0.0f; // Reset bias

    PY[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    PY[0][1] = 0.0f;
    PY[1][0] = 0.0f;
    PY[1][1] = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngleY(float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rateY = newRate - biasY;
    angleY += dt * rateY;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    PY[0][0] += dt * (dt*PY[1][1] - PY[0][1] - PY[1][0] + Q_angleY);
    PY[0][1] -= dt * PY[1][1];
    PY[1][0] -= dt * PY[1][1];
    PY[1][1] += Q_biasY * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = PY[0][0] + R_measureY; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = PY[0][0] / S;
    K[1] = PY[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angleY; // Angle difference
    /* Step 6 */
    angleY += K[0] * y;
    biasY += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = PY[0][0];
    float P01_temp = PY[0][1];

    PY[0][0] -= K[0] * P00_temp;
    PY[0][1] -= K[0] * P01_temp;
    PY[1][0] -= K[1] * P00_temp;
    PY[1][1] -= K[1] * P01_temp;

    return angleY;
};

void setAngleY(float newAngle) { angleY = newAngle; }; // Used to set angle, this should be set as the starting angle
float getRateY(void) { return rateY; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangleY(float newQ_angle) { Q_angleY = newQ_angle; };
void setQbiasY(float newQ_bias) { Q_biasY = newQ_bias; };
void setRmeasureY(float newR_measure) { R_measureY = newR_measure; };

float getQangleY(void) { return Q_angleY; };
float getQbiasY(void) { return Q_biasY; };
float getRmeasureY(void) { return R_measureY; };
