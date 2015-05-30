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

#if USE_SONAR

#include "EEPROM.h"
#include "IMU.h"
#include "Sonar.h"
#include "Time.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
//#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

#define SYSCTL_PERIPH_TRIG          SYSCTL_PERIPH_GPIOE
#define GPIO_SONAR_TRIG_BASE        GPIO_PORTE_BASE
#define GPIO_SONAR_TRIG             GPIO_PIN_0

#define SYSCTL_PERIPH_ECHO          SYSCTL_PERIPH_GPIOB
#define GPIO_SONAR_ECHO_BASE        GPIO_PORTB_BASE
#define GPIO_SONAR_ECHO             GPIO_PIN_2

// Timer used to measure the width of the sonar echo pulse
#define SYSCTL_PERIPH_SONAR_TIMER   SYSCTL_PERIPH_TIMER3
#define GPIO_SONAR_ALTERNATE        GPIO_PB2_T3CCP0
#define SONAR_TIMER_BASE            TIMER3_BASE
#define SONAR_TIMER_INT             INT_TIMER3A

// Implemented based on: http://che126.che.caltech.edu/28015-PING-Sensor-Product-Guide-v2.0.pdf

static volatile int32_t sonarDistanceDeciUs;

static void SonarHandler(void) {
    static uint32_t prev = 0;
    static bool last_edge = false;

    TimerIntClear(SONAR_TIMER_BASE, TIMER_CAPA_EVENT); // Clear interrupt
    uint32_t curr = TimerValueGet(SONAR_TIMER_BASE, TIMER_A); // Read capture value
    bool edge = GPIOPinRead(GPIO_SONAR_ECHO_BASE, GPIO_SONAR_ECHO); // Read the GPIO pin

    if (last_edge && !edge) { // Check that we are going from a positive to falling edge
        if (curr > prev) { // Take care of timer overflow
            uint32_t diff = curr - prev; // Calculate diff
            sonarDistanceDeciUs = 10000000UL / (SysCtlClockGet() / diff); // Convert to deci-us
            //UARTprintf("%u %d %d\n", diff, sonarDistanceDeciUs, sonarDistanceDeciUs / 57);
        }
    }

    prev = curr; // Store previous value
    last_edge = edge; // Store last edge
}

bool triggerSonar(void) {
    static uint32_t lastTrigger = 0;

    uint32_t now = millis();
    if ((int32_t)(now - lastTrigger) > 25) { // Trigger every 25ms
        lastTrigger = now;
        GPIOPinWrite(GPIO_SONAR_TRIG_BASE, GPIO_SONAR_TRIG, GPIO_SONAR_TRIG); // Set pin high
        delayMicroseconds(10); // Other sources wait 10us
        GPIOPinWrite(GPIO_SONAR_TRIG_BASE, GPIO_SONAR_TRIG, 0); // Set pin low
        //UARTprintf("%d\n", getSonarDistance());
        return true;
    }
    return false;
}

// Returns the distance in mm. Range is 0-3000 mm or -1 if the value is invalid.
#if USE_BARO
int16_t getSonarDistance(angle_t *angle, bmp180_t *bmp180) {
    // Use temperature from BMP180 to compensate for the difference in speed of sound in air due to temperature difference
    const uint8_t US_ROUNDTRIP_CM = 1.0f / (3315.0f + (0.6f * bmp180->temperature)) * 2.0f * 1e5f; // Taken from the datasheet - note that temperature is in 0.1 C units, so the equation had to be multiplied by 10
#else
int16_t getSonarDistance(angle_t *angle) {
    static const uint8_t US_ROUNDTRIP_CM = 58; // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total). Calculated at room temperature
#endif
    int16_t distance = sonarDistanceDeciUs / US_ROUNDTRIP_CM; // The output will actually be in mm, as it is in deci-us
    if (distance > 3000) // Datasheet says 3m is maximum
        return -1;

    if (fmaxf(fabsf(angle->axis.roll), fabsf(angle->axis.pitch)) > cfg.maxAngleInclinationSonar) // Return -1 if it is tilted more than the maximum tilt angle
        return -1;

    distance *= cosf(angle->axis.roll * DEG_TO_RAD) * cosf(angle->axis.pitch * DEG_TO_RAD); // Calculate adjacent side

    return distance;
}

void initSonar(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SONAR_TIMER); // Enable Timer peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ECHO); // Enable GPIO peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TRIG); // Enable GPIO peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinConfigure(GPIO_SONAR_ALTERNATE); // Use alternate function
    GPIOPinTypeTimer(GPIO_SONAR_ECHO_BASE, GPIO_SONAR_ECHO); // Use pin with timer peripheral

    // Split timers and enable timer A event up-count timer
    TimerConfigure(SONAR_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);

    // Configure the Timer A
    TimerControlEvent(SONAR_TIMER_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES); // Interrupt on both edges
    TimerIntRegister(SONAR_TIMER_BASE, TIMER_A, SonarHandler); // Register interrupt handler
    TimerIntEnable(SONAR_TIMER_BASE, TIMER_CAPA_EVENT); // Enable timer capture A event interrupt
    IntPrioritySet(SONAR_TIMER_INT, 0); // Configure Timer interrupt priority as 0
    IntEnable(SONAR_TIMER_INT); // Enable Timer interrupt

    GPIOPinTypeGPIOOutput(GPIO_SONAR_TRIG_BASE, GPIO_SONAR_TRIG); // Set pin as output
    GPIOPinWrite(GPIO_SONAR_TRIG_BASE, GPIO_SONAR_TRIG, 0); // Set pin low

    TimerEnable(SONAR_TIMER_BASE, TIMER_A); // Enable Timer A
}

#endif // USE_SONAR
