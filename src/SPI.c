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

#if USE_FLOW_SENSOR

#include "Time.h"
#include "SPI.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"

static void spiSelect(bool enable) {
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, enable ? 0 : GPIO_PIN_3); // The SS pin is active low
}

void initSPI(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0); // Enable SSI0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIOA peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    // Use alternate function
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3); // Set SS as output
    spiSelect(false);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5); // Use pins with SSI peripheral

    // Configure the SSI to MODE3, 2 MHz, and 8-bit data
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 2e6, 8);
    SSIEnable(SSI0_BASE); // Enable the SSI module

    uint32_t buf;
    while (SSIDataGetNonBlocking(SSI0_BASE, &buf)) {
        // Read any residual data from the SSI port.  This makes sure the receive
        // FIFOs are empty, so we don't read any unwanted junk.  This is done here
        // because the SPI SSI mode is full-duplex, which allows you to send and
        // receive at the same time.  The SSIDataGetNonBlocking function returns
        // "true" when data was returned, and "false" when no data was returned.
        // The "non-blocking" function checks if there is any data in the receive
        // FIFO and does not "hang" if there isn't.
    }
}

void spiWrite(uint8_t regAddr, uint8_t data) {
    spiWriteData(regAddr, &data, 1);
}

void spiWriteData(uint8_t regAddr, uint8_t *data, uint8_t length) {
    spiSelect(true);

    // First send data
    SSIDataPut(SSI0_BASE, regAddr | 0x80); // Indicate write operation
    delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
    for (uint8_t i = 0; i < length; i++)
        SSIDataPut(SSI0_BASE, data[i]); // Write data

    // Response is just thrown away
    uint32_t buf;
    SSIDataGet(SSI0_BASE, &buf);
    for (uint8_t i = 0; i < length; i++)
        SSIDataGet(SSI0_BASE, &buf);

    spiSelect(false);
}

uint8_t spiRead(uint8_t regAddr) {
    uint8_t data;
    spiReadData(regAddr, &data, 1);
    return data;
}

void spiReadData(uint8_t regAddr, uint8_t *data, uint8_t length) {
    spiSelect(true);

    // First send data
    SSIDataPut(SSI0_BASE, regAddr); // Send register address
    delayMicroseconds(75); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
    for (uint8_t i = 0; i < length; i++)
        SSIDataPut(SSI0_BASE, 0); // Send 0

    // Read the received response
    uint32_t buf;
    SSIDataGet(SSI0_BASE, &buf); // The first data is just thrown away
    for (uint8_t i = 0; i < length; i++) {
        SSIDataGet(SSI0_BASE, &buf); // Receive rest of the data
        data[i] = buf;
    }

    spiSelect(false);
}

#endif // USE_FLOW_SENSOR
