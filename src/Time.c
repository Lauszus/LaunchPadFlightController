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

#include "Time.h"

#include "driverlib/sysctl.h"
#include "driverlib/systick.h"

static volatile uint32_t counter;

static void SycTickHandler(void) {
    counter++;
}

void initTime(void) {
    SysTickPeriodSet(SysCtlClockGet() / 1000000UL); // 1000 for milliseconds & 1000000 for microseconds
    SysTickIntRegister(SycTickHandler);
    SysTickIntEnable();
    SysTickEnable();
}

void delay(uint32_t ms) {
    delayMicroseconds(ms * 1000UL);
}

void delayMicroseconds(uint32_t us) {
    uint32_t start = micros();
    while ((int32_t)(micros() - start) < us) {
        // Do nothing
    };
}

uint32_t millis(void) {
    return counter / 1000UL;
}

uint32_t micros(void) {
    return counter;
}
