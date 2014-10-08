#include <stdint.h>
#include <stdbool.h>

#include "UART.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

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
