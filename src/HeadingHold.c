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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#if USE_MAG

#include "EEPROM.h"
#include "HeadingHold.h"
#include "Pins.h"

#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

static float magHold; // Heading using for heading hold

float updateHeadingHold(angle_t *angle, float rudder) {
    static const uint8_t headMaxAngle = 25;
    if (fmaxf(fabsf(angle->axis.roll), fabsf(angle->axis.pitch)) < headMaxAngle) { // Check that we are not tilted too much
        float diff = angle->axis.yaw - magHold;
        if (diff < -180.0f) // Convert range back to [-180:180]
            diff += 360.0f;
        if (diff > 180.0f)
            diff -= 360.0f;
        rudder -= diff * cfg.headKp;
        GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED
    } else
        GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED

    return rudder;
}

void resetHeadingHold(angle_t *angle) {
    GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED
    magHold = angle->axis.yaw; // Reset heading hold value
}

#endif // USE_MAG
