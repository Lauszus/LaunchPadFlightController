/* Copyright (C) 2017 Kristian Sloth Lauszus. All rights reserved.

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

#ifndef __low_pass_filter__
#define __low_pass_filter__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float prevOutput, Fc; // Previous output and cutoff frequency in Hz
} low_pass_t;

float applyLowPass(low_pass_t *low_pass, float input, float dt);

#ifdef __cplusplus
}
#endif

#endif
