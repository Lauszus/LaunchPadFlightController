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

#include "KalmanX.h"

/* Kalman filter variables */
float Q_angleX; // Process noise variance for the accelerometer
float Q_biasX; // Process noise variance for the gyro bias
float R_measureX; // Measurement noise variance - this is actually the variance of the measurement noise

float angleX; // The angle calculated by the Kalman filter - part of the 2x1 state vector
float biasX; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rateX; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

float PX[2][2]; // Error covariance matrix - This is a 2x2 matrix

void KalmanXInit(void) {
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angleX = 0.001f;
    Q_biasX = 0.003f;
    R_measureX = 0.03f;

    angleX = 0.0f; // Reset the angle
    biasX = 0.0f; // Reset bias

    PX[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    PX[0][1] = 0.0f;
    PX[1][0] = 0.0f;
    PX[1][1] = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngleX(float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rateX = newRate - biasX;
    angleX += dt * rateX;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    PX[0][0] += dt * (dt*PX[1][1] - PX[0][1] - PX[1][0] + Q_angleX);
    PX[0][1] -= dt * PX[1][1];
    PX[1][0] -= dt * PX[1][1];
    PX[1][1] += Q_biasX * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = PX[0][0] + R_measureX; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = PX[0][0] / S;
    K[1] = PX[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angleX; // Angle difference
    /* Step 6 */
    angleX += K[0] * y;
    biasX += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = PX[0][0];
    float P01_temp = PX[0][1];

    PX[0][0] -= K[0] * P00_temp;
    PX[0][1] -= K[0] * P01_temp;
    PX[1][0] -= K[1] * P00_temp;
    PX[1][1] -= K[1] * P01_temp;

    return angleX;
};

void setAngleX(float newAngle) { angleX = newAngle; }; // Used to set angle, this should be set as the starting angle
float getRateX(void) { return rateX; }; // Return the unbiased rate

/* These are used to tune the Kalman filter */
void setQangleX(float newQ_angle) { Q_angleX = newQ_angle; };
void setQbiasX(float newQ_bias) { Q_biasX = newQ_bias; };
void setRmeasureX(float newR_measure) { R_measureX = newR_measure; };

float getQangleX(void) { return Q_angleX; };
float getQbiasX(void) { return Q_biasX; };
float getRmeasureX(void) { return R_measureX; };
