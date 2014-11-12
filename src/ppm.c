#include <stdint.h>
#include <stdbool.h>

#include "ppm.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

static uint16_t period;

void initPPM(void) {
	//SysCtlPWMClockSet(SYSCTL_PWMDIV_32); // Set divider to 32
	SysCtlPWMClockSet(SYSCTL_PWMDIV_4); // Set divider to 4

	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOC peripheral
	GPIOPinConfigure(GPIO_PB6_M0PWM0); // Use alternate function
	GPIOPinConfigure(GPIO_PB7_M0PWM1); // Use alternate function
	GPIOPinConfigure(GPIO_PB4_M0PWM2); // Use alternate function
	GPIOPinConfigure(GPIO_PB5_M0PWM3); // Use alternate function
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_4 | GPIO_PIN_5); // Use pin with PWM peripheral

	// Configure the PWM generator for count down mode with immediate updates to the parameters.
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

	// Period is given by (SysClk * period) / divider
	// The period is set to 20ms (50 Hz)
	//period = (SysCtlClockGet() / 1000 * 20) / 32; // 50000
	// The period is set to 2.5ms (400 Hz)
	period = (SysCtlClockGet() / 10000 * 25) / 4; // 50000
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period.
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period); // Set the period.

	// Start the timers in generator 0.
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);

	// Enable the outputs.
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);

	writePPMAllOff();
}

uint16_t getPeriod(void) {
	return period;
}

void writePPMAllOff(void) {
	// Turn of all motors
	for (uint8_t i = 0; i < 4; i++)
		writePPMUs(i, PPM_MIN);
}

// From Arduino source code: https://github.com/arduino/Arduino/blob/ide-1.5.x/hardware/arduino/avr/cores/arduino/WMath.cpp
float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateMotor(uint8_t motor, float value) {
	// Motors are in the range from -100 to 100
	if (value > 100.0f)
		value = 100.0f;
	else if (value < -100.0f)
		value= -100.0f;

	uint16_t motorOutput = map(value, -100, 100, PPM_MIN, PPM_MAX);
	writePPMUs(motor, motorOutput);
}

void updateMotorsAll(float *values) {
	for (uint8_t i = 0; i < 4; i++)
		updateMotor(i, values[i]);
}

void writePPMUs(uint8_t motor, uint16_t us) {
	//writePPMWidth(motor, period * us / 20000); // 50 Hz
	writePPMWidth(motor, period * us / 2500); // 400 Hz
}

void writePPMWidth(uint8_t motor, uint16_t width) {
	PWMPulseWidthSet(PWM0_BASE, motor == 0 ? PWM_OUT_0 : motor == 1 ? PWM_OUT_1 : motor == 2 ? PWM_OUT_2 : PWM_OUT_3, width);
}
