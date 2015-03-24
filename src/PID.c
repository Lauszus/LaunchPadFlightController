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

#include <stdint.h>
#include <stdbool.h>

#include "EEPROM.h"
#include "PID.h"

float updatePID(pid_t *pid, float setPoint, float input, float dt) {
    float error = setPoint - input;
    float pTerm = pid->Kp * error;
    pid->integratedError += error * dt;
    pid->integratedError = constrain(pid->integratedError, -pid->integrationLimit, pid->integrationLimit); // Limit the integrated error - prevents windup
    float iTerm = pid->Ki * pid->integratedError;
    float dTerm = pid->Kd * (error - pid->lastError) / dt;
    pid->lastError = error;
    return pTerm + iTerm + dTerm;
}

void resetPIDError(void) {
    cfg.pidRoll.integratedError = cfg.pidRoll.lastError = 0.0f;
    cfg.pidPitch.integratedError = cfg.pidPitch.lastError = 0.0f;
    cfg.pidYaw.integratedError = cfg.pidYaw.lastError = 0.0f;
}

void setDefaultPIDValues(void) {
    cfg.pidRoll.Kp = 0.2f;
    cfg.pidRoll.Ki = 0.8f;
    cfg.pidRoll.Kd = 0.0f;

    cfg.pidRoll.integrationLimit = 0.6f; // Prevent windup

    cfg.pidPitch = cfg.pidRoll; // Use same PID values for both pitch and roll

    // x2 the values work pretty well - TODO: Fine-tune these
    cfg.pidYaw = cfg.pidRoll;
    cfg.pidYaw.Kp *= 3.0f;
    cfg.pidYaw.Ki *= 3.5f; // I increased this in order for it to stop yawing slowly
    cfg.pidYaw.Kd *= 2.0f;

    resetPIDError();
}
