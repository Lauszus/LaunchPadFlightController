#include <stdint.h>
#include <stdbool.h>

#include "ppm.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

static uint16_t period;

// TODO: Scope output
void initPPM(void) {
	SysCtlPWMClockSet(SYSCTL_PWMDIV_32); // Set divider to 32
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOC peripheral
	GPIOPinConfigure(GPIO_PB6_M0PWM0); // Use altenate function
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6); // Use pin with PWM periphera	

	// Configure the PWM generator for count down mode with immediate updates to the parameters.
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	
	// Period is given by (SysClk * period) / divider
	period = (SysCtlClockGet() / 1000 * 20) / 32; // 50000
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period.

	// Start the timers in generator 0.
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);

	// Enable the outputs.
	//PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT , true);
	
/*
//
    // Allow PWM0 generated interrupts.  This configuration is done to
    // differentiate fault interrupts from other PWM0 related interrupts.
    PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0);

    // Enable the PWM0 LOAD interrupt on PWM0.
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_LOAD);

    // Enable the PWM0 interrupts on the processor (NVIC).
    IntEnable(INT_PWM0_0);
*/
}

uint16_t getPeriod(void) {
	return period;
}

void writePPMUs(uint16_t us) {
	writePPMWidth(period * us / 20000);
}

void writePPMWidth(uint16_t width) {
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, width);
}
