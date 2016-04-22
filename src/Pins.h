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

#ifndef __pins_h__
#define __pins_h__

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_LED_BASE       GPIO_PORTF_BASE
#define GPIO_RED_LED        GPIO_PIN_1
#define GPIO_BLUE_LED       GPIO_PIN_2
#define GPIO_GREEN_LED      GPIO_PIN_3

#define GPIO_KEY_BASE       GPIO_PORTF_BASE
#define GPIO_KEY1           GPIO_PIN_4
#define GPIO_KEY2           GPIO_PIN_0

#ifdef __cplusplus
}
#endif

#endif
