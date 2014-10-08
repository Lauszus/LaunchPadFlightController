#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.c"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

#if 0
#define GPIO_RX_PERIPH		SYSCTL_PERIPH_GPIOB // Test code for switch 1
#define GPIO_RX_BASE      GPIO_PORTB_BASE
#define GPIO_RX_PIN       GPIO_PIN_5
#else
#define GPIO_RX_PERIPH		SYSCTL_PERIPH_GPIOF
#define GPIO_RX_BASE      GPIO_PORTF_BASE
#define GPIO_RX_PIN       GPIO_PIN_4
#endif

#define delay(ms)         (SysCtlDelay(SysCtlClockGet() / 3000UL * (ms - 1))) // Delay macro used to set a delay in ms

volatile uint32_t millis;

void SycTickHandler() {
	millis++;
}

void SysTickbegin() {
	SysTickPeriodSet(SysCtlClockGet() / 100000); // 1000 for miliseconds - TODO: Set back to 1000
	SysTickIntRegister(SycTickHandler);
	SysTickIntEnable();
	SysTickEnable();
}

void initUART(void) {
	// Enable the GPIO port containing the pins that will be used.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Configure the GPIO pin muxing for the UART function.
	// This is only necessary if your part supports GPIO pin function muxing.
	// Study the data sheet to see which functions are allocated per pin.
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

  // Since GPIO A0 and A1 are used for the UART function, they must be
  // configured for use as a peripheral function (instead of GPIO).
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTStdioConfig(0, 115200, SysCtlClockGet());
	UARTEchoSet(false);
}

static const uint8_t MAX_CHANNELS = 6;
static volatile uint16_t rxChannel[MAX_CHANNELS];

// TODO: Check that there is 6 valid channels
void Timer1Handler(void) {
	static uint8_t channelIndex = 0;
	static uint32_t prev = 0;

    TimerIntClear(WTIMER1_BASE, TIMER_CAPA_EVENT); // Clear interrupt

	uint32_t curr = TimerValueGet(WTIMER1_BASE, TIMER_A); // Read capture value
	uint32_t diff = curr - prev; // Calculate diff
	uint32_t diff_us = 1000000UL / (SysCtlClockGet() / diff); // Convert to us
    prev = curr; // Store previous value

	// TODO: Should I just change witch egde it triggers on?
	static bool last_edge = false;
	bool edge = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6);

	if (last_edge && !edge) { // Check that we are going from a positive to falling egde
#if 0
		UARTprintf("Channel: %u %u %d\n", period, diff_us,  millis);
		millis = 0;
#else
		if (diff_us > 2700) { // Check if sync pulse is received - see: https://github.com/multiwii/baseflight/blob/master/src/drv_pwm.c
			channelIndex = 0; // Reset channel index
#if 1
			for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
				if (rxChannel[i] > 0)
					UARTprintf("%u\t", rxChannel[i]);
				else
					break;
			}
			UARTprintf("\n");
#endif
		} else {
			rxChannel[channelIndex++] = diff_us;
			if (channelIndex >= MAX_CHANNELS)
				channelIndex = 0;
		}
#endif
	}

	last_edge = edge;
}

void initTimer(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1); // Enable Wide Timer1 peripheral

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable GPIOC peripheral
	GPIOPinConfigure(GPIO_PC6_WT1CCP0); // Use altenate function
	GPIOPinTypeTimer(GPIO_PORTC_BASE, GPIO_PIN_6); // Use pin with timer peripheral

	// TODO: Don't use wide timer and cleanup code

	// Configure and enable  WTimer1A
	TimerConfigure(WTIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP); // Split timers and enable timer A event up-count timer
	TimerControlEvent(WTIMER1_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES); // TIMER_EVENT_POS_EDGE
	//TimerLoadSet(WTIMER1_BASE, TIMER_A, SysCtlClockGet() / 40); // Period = 40 Hz --> 25msec
	TimerIntRegister(WTIMER1_BASE, TIMER_A, Timer1Handler); // Register interrupt handler
	TimerIntEnable(WTIMER1_BASE, TIMER_CAPA_EVENT); // Enable timer capture A event interrupt
	IntEnable(INT_WTIMER1A); // Enable wide Timer 1A interrupt
	TimerEnable(WTIMER1_BASE, TIMER_A); // Enable Timer 1A
}

void RxHandler(void) {
	GPIOIntClear(GPIO_RX_BASE, GPIO_RX_PIN); // Clear interrupt source
/*
	UARTprintf("Millis: %d\n", millis);
	millis = 0;
*/
}

void initIO(void) {
	SysCtlPeripheralEnable(GPIO_RX_PERIPH); // Enable GPIO peripheral

	GPIOPinTypeGPIOInput(GPIO_RX_BASE, GPIO_RX_PIN); // Set as input
#if GPIO_RX_BASE == GPIO_PORTF_BASE && (GPIO_RX_PIN == GPIO_PIN_0 || GPIO_RX_PIN == GPIO_PIN_4) // Check if using built-in switch
	GPIO_PORTF_PUR_R |= GPIO_RX_PIN; // Enable pull-up
#endif

	GPIOIntTypeSet(GPIO_RX_BASE, GPIO_RX_PIN, GPIO_BOTH_EDGES); // Enable interrupt on both falling and rising edge
	GPIOIntRegister(GPIO_RX_BASE, RxHandler); // Register interrupt handler
	GPIOIntEnable(GPIO_RX_BASE, GPIO_RX_PIN); // Enable interrupt
}

int main(void) {
	// Set the clocking to run directly from the external crystal/oscillator.
	//SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set clock to 80MHz (400MHz(PLL) / 2 / 2.5 = 80 MHz)

	initUART();
	SysTickbegin();
	initTimer();
	initIO();

	IntMasterEnable();

	UARTprintf("Started\r\n");
	UARTprintf("CLK %d\r\n", SysCtlClockGet());

	while (1) {
	}
}

// TODO:
// Compare CPPM values with measured values from oscilloscope
// SPI/MPU-6050
// PPM output signal
// EEPROM
