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

#if !defined(__magnetometer_h__) && USE_MAG
#define __magnetometer_h__

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

void initMag(void);
bool getMagData(sensor_t *mag, bool calibrating);
void calibrateMag(void);

#ifdef __cplusplus
}
#endif

#endif
