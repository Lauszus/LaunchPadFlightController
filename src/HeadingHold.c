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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#if USE_MAG

#include "EEPROM.h"
#include "HeadingHold.h"
#include "StepResponse.h"
#include "RX.h"

#if !(STEP_ACRO_SELF_LEVEL || STEP_ALTITUDE_HOLD || STEP_HEADING_HOLD) // The logger will use the blue LED as indicator instead
#include "Config.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#endif

static float magHold; // Heading using for heading hold

float updateHeadingHold(angle_t *angle, float rudder, uint32_t __attribute__((unused)) now) {
    static const uint8_t headMaxAngle = 25;
    if (fmaxf(fabsf(angle->axis.roll), fabsf(angle->axis.pitch)) < headMaxAngle) { // Check that we are not tilted too much
#if STEP_HEADING_HOLD
        static const float step1 = 0; // Start at 0 degrees (North)
        static const float step2 = 45; // Rotate 45 degrees (East)
        static const uint32_t interval = 10e6; // 10 seconds between steps
        magHold = stepResponse(getRXChannel(RX_AUX2_CHAN) > 0, magHold, angle->axis.yaw, step1, step2, interval, now);
#endif
        float error = magHold - angle->axis.yaw;
        if (error < -180.0f) // Normalize difference, so 0 is forward and -180 and 180 is backward
            error += 360.0f;
        else if (error > 180.0f)
            error -= 360.0f;
        rudder += error * cfg.headKp; // Add the output from the P controller to the rudder input
#if !(STEP_ACRO_SELF_LEVEL || STEP_ALTITUDE_HOLD || STEP_HEADING_HOLD)
        GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED
#endif
    }
#if !(STEP_ACRO_SELF_LEVEL || STEP_ALTITUDE_HOLD || STEP_HEADING_HOLD)
    else
        GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED
#endif

    return rudder;
}

void resetHeadingHold(angle_t *angle) {
#if !(STEP_ACRO_SELF_LEVEL || STEP_ALTITUDE_HOLD || STEP_HEADING_HOLD)
    GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED
#endif
    magHold = angle->axis.yaw; // Reset heading hold value
}

#endif // USE_MAG
