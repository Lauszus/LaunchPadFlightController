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
#include <stdlib.h>

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

#define ACRO_MODE 1

int main(void) {
	// Set the clocking to run directly from the external crystal/oscillator.
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set clock to 80MHz (400MHz(PLL) / 2 / 2.5 = 80 MHz)

	initPPM();
	initUART();
	delay(100);
	UARTprintf("Started\n");
	initTime();
	initRX();

	//initMPU6500();
	initMPU6500_i2c();

	IntMasterEnable();

	delay(100); // Wait a little for everything to initialize

	UARTprintf("CLK %d\n", SysCtlClockGet());
	UARTprintf("min: %d, max: %d, period: %d\n", PPM_MIN, PPM_MAX, getPeriod());

#if !ACRO_MODE
	const float restAngleRoll = 1.67f, restAnglePitch = -2.55f; // TODO: Make a calibration routine for these values
#endif

#if ACRO_MODE
	// 0.0200.0.1000.0.0000
	pidRoll.Kp = 0.02f;
	pidRoll.Ki = 0.10f;
	pidRoll.Kd = 0.0f;
#else
	pidRoll.Kp = 1.75f;
	pidRoll.Ki = 1.0f;//2.3f;
	pidRoll.Kd = 0.0f;
#endif
	pidRoll.integratedError = 0.0f;
	pidRoll.lastError = 0.0f;

	pidPitch = pidRoll; // Use same PID values for both pitch and roll
	pidYaw = pidRoll;

	// x2 the values work pretty well - TODO: Fine-tune these
	pidYaw.Kp *= 2.0f;
	pidYaw.Ki *= 2.0f;
	pidYaw.Kd *= 2.0f;
	
	printPIDValues();

	static uint32_t timer = 0;

	// Motor 0 is bottom right, motor 1 is top right, motor 2 is bottom left and motor 3 is top left
	static float motors[4] = { -100.0f, -100.0f, -100.0f, -100.0f };
	static float rollGain = 1.0f, pitchGain = 1.0f, yawGain = 1.0f;

	while (1) {
		checkUARTData();
#if 1
		// TODO: Arm using throttle low and yaw right
		if (!validRXData || rxChannel[RX_AUX1_CHAN] < 1000 || rxChannel[RX_THROTTLE_CHAN] < RX_MIN_INPUT + 25) {
			writePPMAllOff();
			pidRoll.integratedError = 0.0f;
			pidRoll.lastError = 0.0f;
			pidPitch.integratedError = 0.0f;
			pidPitch.lastError = 0.0f;
			pidYaw.integratedError = 0.0f;
			pidYaw.lastError = 0.0f;
		} else if (dataReadyMPU6500()) {
			float dt = (float)(micros() - timer) / 1000000.0f;
			//UARTprintf("%d\n", micros() - timer);
			timer = micros();

#if ACRO_MODE
			int16_t gyroData[3];
			getMPU6500Gyro(gyroData);

			/*UARTprintf("%d\t%d\t%d\n", gyroData[0], gyroData[1], gyroData[2]);
			UARTFlushTx(false);*/
#else
			float roll, pitch;
			getMPU6500Angles(&roll, &pitch, dt);

			/*UARTprintf("%d.%02d\t%d.%02d\n", (int16_t)roll, (int16_t)abs(roll * 100.0f) % 100, (int16_t)pitch, (int16_t)abs(pitch * 100.0f) % 100);
			UARTFlushTx(false);*/
#endif

#if ACRO_MODE
			float rollOut = updatePID(&pidRoll, 0, gyroData[1], dt);
			float pitchOut = updatePID(&pidPitch, 0, gyroData[0], dt);
#else
			float rollOut = updatePID(&pidRoll, restAngleRoll, roll, dt);
			float pitchOut = updatePID(&pidPitch, restAnglePitch, pitch, dt);
#endif
			float yawOut = updatePID(&pidYaw, 0, gyroData[2], dt);

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

			motors[0] -= yawOut * yawGain;
			motors[1] += yawOut * yawGain;
			motors[2] += yawOut * yawGain;
			motors[3] -= yawOut * yawGain;

			for (uint8_t i = 0; i < 4; i++)
				motors[i] = constrain(motors[i], -50.0f, 50.0f);

			// Pitch Control
			float elevator = map(rxChannel[RX_ELEVATOR_CHAN], RX_MIN_INPUT, RX_MAX_INPUT, -100.0f, 100.0f);
			motors[0] += elevator;
			motors[1] -= elevator;
			motors[2] += elevator;
			motors[3] -= elevator;

			// Roll Control
			float aileron = map(rxChannel[RX_AILERON_CHAN], RX_MIN_INPUT, RX_MAX_INPUT, -100.0f, 100.0f);
			motors[0] -= aileron;
			motors[1] -= aileron;
			motors[2] += aileron;
			motors[3] += aileron;

			// Rudder Control
			float rudder = map(rxChannel[RX_RUDDER_CHAN], RX_MIN_INPUT, RX_MAX_INPUT, -100.0f, 100.0f);
			motors[0] -= rudder;
			motors[1] += rudder;
			motors[2] += rudder;
			motors[3] -= rudder;

			updateMotorsAll(motors);

			//UARTprintf("%d\t%d\n", (int16_t)elevator, (int16_t)aileron);
#if 0
			UARTprintf("%d\t%d\t\t", (int16_t)roll, (int16_t)pitch);
			UARTprintf("%d\t%d\t\t", (int16_t)rollOut, (int16_t)pitchOut);
			UARTprintf("%d\t%d\t%d\t%d\n", (int16_t)motors[0], (int16_t)motors[1], (int16_t)motors[2], (int16_t)motors[3]);
			UARTFlushTx(false);
#endif
		}
#endif
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
	// Save PID values in EEPROM
	// Adjust PID values using pots on transmitter
	// Only enable peripheral once
	// Tune yaw PID values separately
	// Should I limit loop time to 2.5ms (400Hz)?
	// Make limit of integrated error adjustable
