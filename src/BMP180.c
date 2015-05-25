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

#if USE_BARO

#include <stdint.h>
#include <stdbool.h>

#include "BMP180.h"
#include "I2C.h"

#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif


// Returns true when data is ready to be read
bool dataReadyBMP180(void) {
    return true;
}

static void getBMP180DataRaw(sensorRaw_t *baroRaw) {
    /*uint8_t buf[6];
    i2cReadData(HMC5883L_ADDRESS, HMC5883L_OUTPUT_REG_X_MSB, buf, 6); // Get magnetometer values

    baroRaw->X = (buf[0] << 8) | buf[1];
    baroRaw->Z = (buf[2] << 8) | buf[3];
    baroRaw->Y = (buf[4] << 8) | buf[5];*/
}

void getBMP180Data(bmp180_t *bmp180, bool calibrating) {
    /*getHMC5883LDataRaw(&hmc5883l->magRaw); // Get raw reading
    for (uint8_t axis = 0; axis < 3; axis++)
        hmc5883l->mag.data[axis] = (float)hmc5883l->magRaw.data[axis] * hmc5883l->magGain.data[axis]; // Apply gain

    hmc5883lBoardOrientation(&hmc5883l->mag); // Apply board orientation

    if (!calibrating) { // If we are not calibrating, then subtract zero values
        for (uint8_t axis = 0; axis < 3; axis++)
            hmc5883l->mag.data[axis] -= cfg.magZero.data[axis]; // Subtract zero value stored in EEPROM
    }*/

#if 0 && UART_DEBUG
    UARTprintf("%d.%03u\t%d.%03u\t%d.%03u\n",
                                            (int16_t)hmc5883l->mag.X, (uint16_t)(abs(hmc5883l->mag.X * 1000.0f) % 1000),
                                            (int16_t)hmc5883l->mag.Y, (uint16_t)(abs(hmc5883l->mag.Y * 1000.0f) % 1000),
                                            (int16_t)hmc5883l->mag.Z, (uint16_t)(abs(hmc5883l->mag.Z * 1000.0f) % 1000));
    UARTFlushTx(false);
#endif
}

// Inspired by: https://code.google.com/p/open-headtracker and https://github.com/cleanflight/cleanflight/blob/master/src/main/drivers/compass_hmc5883l.c
void intBMP180(bmp180_t *bmp180) {
#if 0 && UART_DEBUG
    UARTprintf("Mag cal: %d\t%d\t%d\t", (int16_t)mag_total.X, (int16_t)mag_total.Y, (int16_t)mag_total.Z);
    UARTprintf("Gain: %d.%03u\t%d.%03u\t%d.%03u\n",
                                                    (int16_t)hmc5883l->magGain.X, (uint16_t)(abs(hmc5883l->magGain.X * 1000.0f) % 1000),
                                                    (int16_t)hmc5883l->magGain.Y, (uint16_t)(abs(hmc5883l->magGain.Y * 1000.0f) % 1000),
                                                    (int16_t)hmc5883l->magGain.Z, (uint16_t)(abs(hmc5883l->magGain.Z * 1000.0f) % 1000));
    UARTFlushTx(false);
#endif
}

#endif // USE_BARO
