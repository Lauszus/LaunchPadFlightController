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

#if !defined(__logger_h__) && (STEP_ACRO_SELF_LEVEL || STEP_ALTITUDE_HOLD || STEP_HEADING_HOLD)
#define __logger_h__

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

float stepResponse(bool active, float setPoint, float input, float step1, float step2, uint32_t interval, uint32_t now);

#ifdef __cplusplus
}
#endif

#endif
