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

#include "Buzzer.h"
#include "Time.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#define SYSCTL_PERIPH_BUZZER    SYSCTL_PERIPH_GPIOD
#define GPIO_BUZZER_BASE        GPIO_PORTD_BASE
#define GPIO_PIN_BUZZER         GPIO_PIN_2

void initBuzzer(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_BUZZER); // Enable peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinTypeGPIOOutput(GPIO_BUZZER_BASE, GPIO_PIN_BUZZER);
}

inline void buzzer(bool enable) {
    GPIOPinWrite(GPIO_BUZZER_BASE, GPIO_PIN_BUZZER, enable ? GPIO_PIN_BUZZER : 0);
}

void beepBuzzer(void) {
    buzzer(true);
    delay(100);
    buzzer(false);
}

void beepLongBuzzer(void) {
    buzzer(true);
    delay(1000);
    buzzer(false);
}
