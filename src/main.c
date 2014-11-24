/* Copyright (C) 2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

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

#include "RX.h"
#include "UART.h"
#include "time.h"
#include "PPM.h"
#include "PID.h"
#include "MPU6500.h"

#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

int main(void) {
	// Set the clocking to run directly from the external crystal/oscillator.
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set clock to 80MHz (400MHz(PLL) / 2 / 2.5 = 80 MHz)

	initUART();
	delay(100);
	UARTprintf("Started\n");
	initTime();
	initRX();
	initPPM();

	//initMPU6500();
	initMPU6500_i2c();

	IntMasterEnable();

	delay(100); // Wait a little for everything to initialize

	UARTprintf("CLK %d\n", SysCtlClockGet());
	UARTprintf("min: %d, max: %d, period: %d\n", PPM_MIN, PPM_MAX, getPeriod());

	pid_t pidRoll, pidPitch;

	pidRoll.Kp = 0.5f;
	pidRoll.Ki = 0.0f;
	pidRoll.Kd = 0.0f;
	pidRoll.integratedError = 0.0f;
	pidRoll.lastError = 0.0f;

	pidPitch = pidRoll;

	static uint32_t timer = 0;

	// Motor 0 is bottom right, motor 1 is top right, motor 2 is bottom left and motor 3 is top left
	static float motors[4] = { -100.0f, -100.0f, -100.0f, -100.0f };
	static float rollGain = 1.0f, pitchGain = 1.0f;

	while (1) {
		if (rxChannel[RX_AUX1_CHAN] < 1000)
			writePPMAllOff();
		else if (dateReadyMPU6500()) {
			float dt = (float)(micros() - timer) / 1000000.0f;
			//UARTprintf("%d\n", micros() - timer);
			timer = micros();

			float roll, pitch;
			getMPU6500Angles(&roll, &pitch, dt);
			float rollOut = updatePID(&pidRoll, 0.0f, roll, dt); // TODO: Make set point adjustable
			float pitchOut = updatePID(&pidPitch, 0.0f, pitch, dt);

			float throttle = map(rxChannel[RX_THROTTLE_CHAN], RX_MIN_INPUT, RX_MAX_INPUT, -100.0f, 100.0f);
			for (uint8_t i = 0; i < 4; i++)
				motors[i] = throttle;

			motors[0] -= rollOut * rollGain;
			motors[1] -= rollOut * rollGain;
			motors[2] += rollOut * rollGain;
			motors[3] += rollOut * rollGain;

			motors[0] += pitchOut * pitchGain;
			motors[1] -= pitchOut * pitchGain;
			motors[2] += pitchOut * pitchGain;
			motors[3] -= pitchOut * pitchGain;
/*
			motors[0] -= yawOut * yawGain;
			motors[1] += yawOut * yawGain;
			motors[2] += yawOut * yawGain;
			motors[3] -= yawOut * yawGain;
*/
			updateMotorsAll(motors);
#if 0
			UARTprintf("%d\t%d\t\t", (int16_t)roll, (int16_t)pitch);
			UARTprintf("%d\t%d\t\t", (int16_t)rollOut, (int16_t)pitchOut);
			UARTprintf("%d\t%d\t%d\t%d\n", (int16_t)motors[0], (int16_t)motors[1], (int16_t)motors[2], (int16_t)motors[3]);
			UARTFlushTx(false);
#endif
		}

#if 0
		for (int8_t i = -100; i < 100; i++) {
			updateMotor(0, i);
			UARTprintf("%d\n", i);
			delay(20);
		}
		for (int8_t i = 100; i > -100; i--) {
			updateMotor(0, i);
			UARTprintf("%d\n", i);
			delay(20);
		}
#endif
#if 0
		static uint16_t width = PPM_MIN;
		static bool upCounting = true;

		if (rxChannel[RX_AUX1_CHAN] < 1000) { // Used to stop motor output
			width = PPM_MIN;
			upCounting = true;
		}

		UARTprintf("Width: %d\n", width);
		for (uint8_t i = 0; i < 4; i++)
			writePPMUs(i, width);

		if (upCounting)
			width++;
		else
			width--;
		if (upCounting && width > PPM_MAX)
			upCounting = false;
		else if (!upCounting && width < PPM_MIN)
			upCounting = true;
		delay(5);
#endif
	}
}

// TODO:
	// SPI/MPU-6050
	// Sync all PPM output signals - is this needed?
	// Check PPM output frequency of Naze32
	// EEPROM
	// SysCtlDelay(2);	//insert a few cycles after enabling the peripheral to allow the clock to be fully activated.
	// Adjust PID values using switches
	// Only enable peripheral once
