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

#if !defined(__adns3080_h__) && USE_FLOW_SENSOR
#define __adns3080_h__

#ifdef __cplusplus
extern "C" {
#endif

void initADNS3080(void);
bool dataReadyADNS3080(void);
void getADNS3080Data(int32_t *x, int32_t *y);
void clearMotion(int32_t *x, int32_t *y);

#ifdef __cplusplus
}
#endif

#endif
