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

#ifndef __pid_h__
#define __pid_h__

#ifdef __cplusplus
extern "C" {
#endif

// From Arduino source code
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct {
    float Kp, Ki, Kd; // PID variables
    float integrationLimit;
} pid_values_t;

typedef struct {
    pid_values_t *values; // Use pointer to pid_values_t struct that are saved in the EEPROM
    float iTerm;
    float lastError, deltaError1, deltaError2;
} pid_t;

void initPID(void);
float updatePID(pid_t *pid, float setPoint, float input, float dt);
void resetPIDRollPitchYaw(void);
void resetPIDAltHold(void);

extern pid_t pidRoll, pidPitch, pidYaw, pidSonarAltHold, pidBaroAltHold;

#ifdef __cplusplus
}
#endif

#endif
