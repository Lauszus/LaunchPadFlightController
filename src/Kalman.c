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

#include "Kalman.h"

void KalmanInit(kalman_t *kalman) {
    /* We will set the variables like so, these can also be tuned by the user */
    if (kalman->Q_angle == 0) // Make sure coefficients are not already set
        kalman->Q_angle = 0.001f;
    if (kalman->Q_bias == 0)
        kalman->Q_bias = 0.003f;
    if (kalman->R_measure == 0)
        kalman->R_measure = 0.03f;

    kalman->angle = 0.0f; // Reset the angle
    kalman->bias = 0.0f; // Reset bias

    kalman->P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float getAngle(kalman_t *kalman, float newAngle, float newRate, float dt) {
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = kalman->P[0][0] + kalman->R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - kalman->angle; // Angle difference
    /* Step 6 */
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
};

float getRate(kalman_t *kalman) { return kalman->rate; }; // Return the unbiased rate
float getQangle(kalman_t *kalman) { return kalman->Q_angle; };
float getQbias(kalman_t *kalman) { return kalman->Q_bias; };
float getRmeasure(kalman_t *kalman) { return kalman->R_measure; };

/* These are used to tune the Kalman filter */
void setAngle(kalman_t *kalman, float angle) { kalman->angle = angle; }; // Used to set angle, this should be set as the starting angle
void setQangle(kalman_t *kalman, float Q_angle) { kalman->Q_angle = Q_angle; };
void setQbias(kalman_t *kalman, float Q_bias) { kalman->Q_bias = Q_bias; };
void setRmeasure(kalman_t *kalman, float R_measure) { kalman->R_measure = R_measure; };
