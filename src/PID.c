/* Copyright (C) 2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

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

#include "PID.h"

float updatePID(pid_t *pid, float restAngle, float inputAngle, float dt) {
	/* Update PID values */
	float error = (restAngle - inputAngle);
	float pTerm = pid->Kp * error;
	pid->integratedError += error * dt;
	pid->integratedError = constrain(pid->integratedError, -2.0f, 2.0f); // Limit the integrated error
	float iTerm = pid->Ki * pid->integratedError;
	float dTerm = pid->Kd * (error - pid->lastError) / dt;
	pid->lastError = error;
	return pTerm + iTerm + dTerm;
}
