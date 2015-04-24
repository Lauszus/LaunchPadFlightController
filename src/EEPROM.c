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

#include "Buzzer.h"
#include "EEPROM.h"
#include "Kalman.h"

#include "driverlib/eeprom.h"
#include "driverlib/sysctl.h"
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

static const uint32_t configVersion = 11; // Must be bumped every time config_t is changed
config_t cfg;

void initEEPROM(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); // Enable EEPROM peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    // Make sure config_t is a multiple of 4 - the compiler should pack structs to 4 bytes, but I added this check to be 100% sure
    if (sizeof(config_t) % 4 != 0) {
#if UART_DEBUG
        UARTprintf("Config size error: %u\n", sizeof(config_t));
#endif
        buzzer(true);
        while (1);
    }

    uint32_t rcode = EEPROMInit();
    if (rcode) {
#if UART_DEBUG
        UARTprintf("EEPROMInit error: %u\n", rcode);
#endif
        buzzer(true);
        while (1);
    }

    uint32_t version;
    EEPROMRead(&version, 0, sizeof(version));
    if (version != configVersion) {
        setDefaultConfig();
        beepLongBuzzer();
    } else {
        EEPROMRead((uint32_t*)&cfg, sizeof(configVersion), sizeof(config_t)); // Read config from EEPROM

        /*kalmanRoll.Q_angle = kalmanPitch.Q_angle = cfg.Q_angle; // Set Kalman filter coefficients
        kalmanRoll.Q_bias = kalmanPitch.Q_angle = cfg.Q_bias;
        kalmanRoll.R_measure = kalmanPitch.Q_angle = cfg.R_measure;*/
    }
}

void setDefaultConfig(void) {
    cfg.pidRollValues.Kp = 0.310f;
    cfg.pidRollValues.Ki = 1.65f;
    cfg.pidRollValues.Kd = 0.00040f;
    cfg.pidRollValues.integrationLimit = 5.85f; // Prevent windup

    cfg.pidPitchValues = cfg.pidRollValues; // Use same PID values for both pitch and roll

    cfg.pidYawValues.Kp = 1.000f;
    cfg.pidYawValues.Ki = 6.00f;
    cfg.pidYawValues.Kd = 0.00040f;
    cfg.pidYawValues.integrationLimit = 10.0f; // Prevent windup

    resetPIDTerms();

    cfg.angleKp = 4.70f;
    cfg.maxAngleInclination = 50.0f; // Max angle in self level mode
    cfg.stickScalingRollPitch = 4.69f;
    cfg.stickScalingYaw = 2.0f;

    /*cfg.Q_angle = 0.001f; // Kalman filter coefficients default values
    cfg.Q_bias = 0.003f;
    cfg.R_measure = 0.03f;*/

    cfg.calibrateESCs = false;

    for (uint8_t axis = 0; axis < 3; axis++) {
        cfg.accZero.data[axis] = 0;
        cfg.magZero.data[axis] = 0;
    }

    uint32_t rcode = EEPROMProgram((uint32_t*)&configVersion, 0, sizeof(configVersion)); // Write version number to EEPROM
    if (rcode) {
#if UART_DEBUG
        UARTprintf("Error writing version number to EEPROM: %u\n", rcode);
#endif
        buzzer(true);
    } else
        updateConfig(); // Write values to EEPROM
}

void updateConfig(void) {
    uint32_t rcode = EEPROMProgram((uint32_t*)&cfg, sizeof(configVersion), sizeof(config_t)); // Write config to EEPROM
    if (rcode) {
#if UART_DEBUG
        UARTprintf("Error writing config to EEPROM: %u\n", rcode);
#endif
        buzzer(true);
    } else {
        /*kalmanRoll.Q_angle = kalmanPitch.Q_angle = cfg.Q_angle; // Set Kalman filter coefficients
        kalmanRoll.Q_bias = kalmanPitch.Q_angle = cfg.Q_bias;
        kalmanRoll.R_measure = kalmanPitch.Q_angle = cfg.R_measure;*/
    }
}
