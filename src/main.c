#include <stdint.h>
#include <stdbool.h>

#include "RX.h"
#include "UART.h"

#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
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

#define delay(ms)         (SysCtlDelay(SysCtlClockGet() / 3000UL * (ms - 1))) // Delay macro used to set a delay in ms - TOOD: Explain -1

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
	initRX();
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
