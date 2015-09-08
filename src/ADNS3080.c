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

// Based on: https://github.com/diydrones/ardupilot/tree/5ddbcc296dd6dd9ac9ed6316ac3134c736ae8a78/libraries/AP_OpticalFlow

#include <stdint.h>
#include <stdbool.h>

#if USE_FLOW_SENSOR

#include "ADNS3080.h"
#include "SPI.h"
#include "Time.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_CONFIGURATION_BITS    0x0A
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_MOTION_BURST          0x50

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X              30
#define ADNS3080_PIXELS_Y              30

// Id returned by ADNS3080_PRODUCT_ID register
#define ADNS3080_PRODUCT_ID_VALUE      0x17

#define GPIO_RESET_PERIPH              SYSCTL_PERIPH_GPIOF
#define GPIO_RESET_BASE                GPIO_PORTF_BASE
#define GPIO_RESET_PIN                 GPIO_PIN_4

static void reset(void) {
    GPIOPinWrite(GPIO_RESET_BASE, GPIO_RESET_PIN, GPIO_RESET_PIN); // Set high
    delayMicroseconds(10);
    GPIOPinWrite(GPIO_RESET_BASE, GPIO_RESET_PIN, 0); // Set low
    delayMicroseconds(500); // Wait for sensor to get ready
}

void initADNS3080(void) {
    initSPI();

    // Set reset pin as output
    SysCtlPeripheralEnable(GPIO_RESET_PERIPH); // Enable GPIO peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinTypeGPIOOutput(GPIO_RESET_BASE, GPIO_RESET_PIN); // Set as output
    reset();

    uint8_t id = spiRead(ADNS3080_PRODUCT_ID);
    if (id == ADNS3080_PRODUCT_ID_VALUE) {
#if UART_DEBUG
        UARTprintf("ADNS-3080 found\n");
#endif
    } else {
#if UART_DEBUG
        UARTprintf("Could not find ADNS-3080: %02X\n", id);
#endif
        while (1);
    }

    uint8_t config = spiRead(ADNS3080_CONFIGURATION_BITS);
    spiWrite(ADNS3080_CONFIGURATION_BITS, config | 0x10); // Set resolution to 1600 counts per inch
}

// Run: 'python ADNS3080ImageGrabber.py' in order to see the data
static void __attribute__((unused)) printPixelData(void) {
  bool isFirstPixel = true;

  // Write to frame capture register to force capture of frame
  spiWrite(ADNS3080_FRAME_CAPTURE, 0x83);

  // Wait 3 frame periods + 10 nanoseconds for frame to be captured
  delayMicroseconds(1510); // Minimum frame speed is 2000 frames/second so 1 frame = 500 nano seconds. So 500 x 3 + 10 = 1510

  // Display the pixel data
  for (uint8_t i = 0; i < ADNS3080_PIXELS_Y; i++) {
    for (uint8_t j = 0; j < ADNS3080_PIXELS_X; j++) {
      uint8_t regValue = spiRead(ADNS3080_FRAME_CAPTURE);
      if (isFirstPixel && !(regValue & 0x40)) {
          UARTprintf("Failed to find first pixel: %02X\n", regValue);
          goto reset;
      }
      isFirstPixel = false;
      uint8_t pixelValue = regValue << 2; // Only lower 6 bits contains data
      UARTprintf("%u", pixelValue);
      if (j != ADNS3080_PIXELS_X - 1)
          UARTwrite(",", 1);
    }
    UARTwrite("\n", 1);
    UARTFlushTx(false);
  }

reset:
  reset(); // Hardware reset to restore sensor to normal operation
}

bool dataReadyADNS3080(void) {
    return true;
}

void getADNS3080Data(int32_t __attribute__((unused)) *x, int32_t __attribute__((unused)) *y) {
#if 1
    // Read sensor
    uint8_t buf[4];
    spiReadData(ADNS3080_MOTION_BURST, buf, 4);
    uint8_t motion = buf[0];
    //UARTprintf("%02X\n", motion & 0x01); // Resolution bit

    if (motion & 0x10) // Check if we've had an overflow
        UARTprintf("ADNS-3080 overflow\n");
    else if (motion & 0x80) {
        int8_t dx = buf[1];
        int8_t dy = buf[2];
        uint8_t surfaceQuality = buf[3];

        *x += dx;
        *y += dy;

        // Print values
        UARTprintf("%d,%d\t%d,%d\t%u\n", *x, dx, *y, dy, surfaceQuality);
        UARTFlushTx(false);
    }
#if 0
    else
        UARTprintf("%02X\n", motion);
#endif
#else
    UARTprintf("image data --------------\n");
    printPixelData();
    UARTprintf("-------------------------\n");
    UARTFlushTx(false);
    delay(1500);
#endif
}

// Will cause the Delta_X, Delta_Y, and internal motion registers to be cleared
void clearMotion(int32_t *x, int32_t *y) {
  spiWrite(ADNS3080_MOTION_CLEAR, 0xFF); // Writing anything to this register will clear the sensor's motion registers
  *x = *y = 0;
}

#endif // USE_FLOW_SENSOR
