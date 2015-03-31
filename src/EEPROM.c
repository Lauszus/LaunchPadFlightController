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

#include <stdint.h>
#include <stdbool.h>

#include "EEPROM.h"
#include "Kalman.h"

#include "driverlib/eeprom.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

extern kalman_t kalmanRoll, kalmanPitch; // Structs used for Kalman filter roll and pitch in main.c

const uint32_t configVersion = 5; // Must be bumped every time config_t is changed
config_t cfg;

void setDefaultConfig(void) {
    setDefaultPIDValues();

    cfg.angleKp = 4.0f;
    cfg.stickScalingRollPitch = 2.0f;
    cfg.stickScalingYaw = 2.0f;
    cfg.maxAngleInclination = 50.0f; // Max angle in self level mode

    cfg.Q_angle = 0.001f; // Kalman filter coefficients default values
    cfg.Q_bias = 0.003f;
    cfg.R_measure = 0.03f;

    cfg.calibrateESCs = false;

    for (uint8_t axis = 0; axis < 3; axis++)
        cfg.accZero.data[axis] = 0;

    uint32_t rcode = EEPROMProgram((uint32_t*)&configVersion, 0, sizeof(configVersion)); // Write version number to EEPROM
    if (rcode) {
        UARTprintf("Error writing version number to EEPROM: %u\n", rcode);
        // TODO: Turn buzzer on
    } else
        updateConfig(); // Write values to EEPROM
}

void initEEPROM(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); // Enable EEPROM peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    // Make sure config_t is a multiple of 4 - the compiler should pack struct to 4 bytes, but I added this check to be 100% sure
    if (sizeof(config_t) % 4 != 0) {
        UARTprintf("Config size error: %u\n", sizeof(config_t));
        // TODO: Turn on buzzer
        while (1);
    }

    uint32_t rcode = EEPROMInit();
    if (rcode) {
        UARTprintf("EEPROMInit error: %u\n", rcode);
        // TODO: Turn on buzzer
        while (1);
    }

    uint32_t version;
    EEPROMRead(&version, 0, sizeof(version));
    if (version != configVersion)
        setDefaultConfig();
    else {
        EEPROMRead((uint32_t*)&cfg, sizeof(configVersion), sizeof(config_t)); // Read config from EEPROM

        kalmanRoll.Q_angle = kalmanPitch.Q_angle = cfg.Q_angle; // Set Kalman filter coefficients
        kalmanRoll.Q_bias = kalmanPitch.Q_angle = cfg.Q_bias;
        kalmanRoll.R_measure = kalmanPitch.Q_angle = cfg.R_measure;
    }
}
void updateConfig(void) {
    uint32_t rcode = EEPROMProgram((uint32_t*)&cfg, sizeof(configVersion), sizeof(config_t)); // Write config to EEPROM
    if (rcode) {
        UARTprintf("Error writing config to EEPROM: %u\n", rcode);
        // TODO: Turn buzzer on
    } else {
        kalmanRoll.Q_angle = kalmanPitch.Q_angle = cfg.Q_angle; // Set Kalman filter coefficients
        kalmanRoll.Q_bias = kalmanPitch.Q_angle = cfg.Q_bias;
        kalmanRoll.R_measure = kalmanPitch.Q_angle = cfg.R_measure;
    }
}
