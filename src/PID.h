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

#ifndef __pid_h__
#define __pid_h__

#ifdef __cplusplus
extern "C" {
#endif

// From Arduino source code
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct {
    float Kp, Ki, Kd; // PID variables
    float lastError, integratedError;
    float integrationLimit;
} __attribute__((packed)) pid_t;

float updatePID(pid_t *pid, float setPoint, float input, float dt);
void resetPIDError(void);
void setDefaultPIDValues(void);

#ifdef __cplusplus
}
#endif

#endif
