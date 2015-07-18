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

#if !defined(__ak8963_h__) && USE_MAG
#define __ak8963_h__

#include <stdbool.h>

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    sensorRaw_t mag; // Magnetometer readings with offset subtracted
} ak8963_t;

bool initAK8963(/*ak8963_t *ak8963*/);
bool dataReadyAK8963(void);
void getAK8963Data(ak8963_t *ak8963, bool calibrating);

#ifdef __cplusplus
}
#endif

#endif
