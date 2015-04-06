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
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

extern kalman_t kalmanRoll, kalmanPitch; // Structs used for Kalman filter roll and pitch in main.c

static const uint32_t configVersion = 6; // Must be bumped every time config_t is changed
config_t cfg;

static void setDefaultConfig(void) {
    cfg.pidRoll.Kp = 0.2f;
    cfg.pidRoll.Ki = 0.8f;
    cfg.pidRoll.Kd = 0.0f;
    cfg.pidRoll.integrationLimit = 0.6f; // Prevent windup

    cfg.pidPitch = cfg.pidRoll; // Use same PID values for both pitch and roll

    // x2 the values work pretty well - TODO: Fine-tune these
    cfg.pidYaw = cfg.pidRoll;
    cfg.pidYaw.Kp *= 3.0f;
    cfg.pidYaw.Ki *= 3.5f; // I increased this in order for it to stop yawing slowly
    cfg.pidYaw.Kd *= 2.0f;

    resetPIDError();

    cfg.angleKp = 4.0f;
    cfg.maxAngleInclination = 50.0f; // Max angle in self level mode
    cfg.stickScalingRollPitch = 2.0f;
    cfg.stickScalingYaw = 2.0f;

    cfg.Q_angle = 0.001f; // Kalman filter coefficients default values
    cfg.Q_bias = 0.003f;
    cfg.R_measure = 0.03f;

    cfg.calibrateESCs = false;

    for (uint8_t axis = 0; axis < 3; axis++)
        cfg.accZero.data[axis] = 0;

    uint32_t rcode = EEPROMProgram((uint32_t*)&configVersion, 0, sizeof(configVersion)); // Write version number to EEPROM
    if (rcode) {
#if UART_DEBUG
        UARTprintf("Error writing version number to EEPROM: %u\n", rcode);
#endif
        // TODO: Turn buzzer on
    } else
        updateConfig(); // Write values to EEPROM
}

void initEEPROM(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); // Enable EEPROM peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    // Make sure config_t is a multiple of 4 - the compiler should pack structs to 4 bytes, but I added this check to be 100% sure
    if (sizeof(config_t) % 4 != 0) {
#if UART_DEBUG
        UARTprintf("Config size error: %u\n", sizeof(config_t));
#endif
        // TODO: Turn on buzzer
        while (1);
    }

    uint32_t rcode = EEPROMInit();
    if (rcode) {
#if UART_DEBUG
        UARTprintf("EEPROMInit error: %u\n", rcode);
#endif
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
#if UART_DEBUG
        UARTprintf("Error writing config to EEPROM: %u\n", rcode);
#endif
        // TODO: Turn buzzer on
    } else {
        kalmanRoll.Q_angle = kalmanPitch.Q_angle = cfg.Q_angle; // Set Kalman filter coefficients
        kalmanRoll.Q_bias = kalmanPitch.Q_angle = cfg.Q_bias;
        kalmanRoll.R_measure = kalmanPitch.Q_angle = cfg.R_measure;
    }
}
