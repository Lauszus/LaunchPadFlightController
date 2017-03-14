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

#include <stdint.h>
#include <stdbool.h>

#include "EEPROM.h"
#include "PID.h"
#include "Types.h"

pid_t pidRoll, pidPitch, pidYaw, pidSonarAltHold, pidBaroAltHold;

void initPID(void) {
    // Set PID values to point at values read from EEPROM
    pidRoll.values = &cfg.pidRollPitchValues;
    pidPitch.values = &cfg.pidRollPitchValues;
    pidYaw.values = &cfg.pidYawValues;
    pidSonarAltHold.values = &cfg.pidSonarAltHoldValues;
    pidBaroAltHold.values = &cfg.pidBaroAltHoldValues;
}

float updatePID(pid_t *pid, float setPoint, float input, float dt) {
    float error = setPoint - input;

    // P-term
    float pTerm = pid->values->Kp * error;

    // I-term
    // Use Trapezoidal Integration, see: http://ecee.colorado.edu/shalom/Emulations.pdf
    pid->iTerm += pid->values->Ki * (error + pid->lastError) / 2.0f * dt; // Multiplication with Ki is done before integration limit, to make it independent from integration limit value
    pid->iTerm = constrain(pid->iTerm, -pid->values->integrationLimit, pid->values->integrationLimit); // Limit the integrated error - prevents windup

    // D-term
    // Use exponential smoothing: https://en.wikipedia.org/wiki/Exponential_smoothing
    float deltaError = (error - pid->lastError) / dt; // Calculate difference and compensate for difference in time by dividing by dt
    pid->lastError = error;
    float tau = 1.0f/(2.0f*M_PIf*pid->values->Fc); // Time constant
    float alpha = dt/(tau + dt); // See (10) in http://techteach.no/simview/lowpass_filter/doc/filter_algorithm.pdf
    pid->derivative = pid->derivative + alpha * (deltaError - pid->derivative); // y(n) = y(n-1) + alpha*(u(n) - y(n-1))
    float dTerm = pid->values->Kd * pid->derivative;

    return pTerm + pid->iTerm + dTerm; // Return sum
}

void resetPIDRollPitchYaw(void) {
    pidRoll.iTerm = pidRoll.lastError = pidRoll.derivative = 0.0f;
    pidPitch.iTerm = pidPitch.lastError = pidPitch.derivative = 0.0f;
    pidYaw.iTerm = pidYaw.lastError = pidYaw.derivative = 0.0f;
}

void resetPIDAltHold(void) {
    pidSonarAltHold.iTerm = pidSonarAltHold.lastError = pidSonarAltHold.derivative = 0.0f;
    pidBaroAltHold.iTerm = pidBaroAltHold.lastError = pidBaroAltHold.derivative = 0.0f;
}
