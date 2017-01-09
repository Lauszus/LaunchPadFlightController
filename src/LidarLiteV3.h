/* Copyright (C) 2016 Kristian Sloth Lauszus. All rights reserved.

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

#if !defined(__lidarlitev3_h__) && USE_LIDAR_LITE
#define __lidarlitev3_h__

#ifdef __cplusplus
extern "C" {
#endif

#include "Types.h"

void initLidarLite(void);
bool triggerLidarLite(void);
int32_t getLidarLiteDistance(angle_t *angle);

#ifdef __cplusplus
}
#endif

#endif
