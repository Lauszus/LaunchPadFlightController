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

#include "Buzzer.h"
#include "EEPROM.h"

#include "driverlib/eeprom.h"
#include "driverlib/sysctl.h"
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

static const uint32_t configVersion = 19; // Must be bumped every time config_t is changed
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
    } else
        EEPROMRead((uint32_t*)&cfg, sizeof(configVersion), sizeof(config_t)); // Read config from EEPROM
}

void setDefaultConfig(void) {
    cfg.pidRollPitchValues.Kp = 0.310f;
    cfg.pidRollPitchValues.Ki = 1.65f;
    cfg.pidRollPitchValues.Kd = 0.00040f;
    cfg.pidRollPitchValues.integrationLimit = 5.85f; // Prevent windup
    cfg.pidRollPitchValues.Fc = 100.0f; // 100 Hz

    cfg.pidYawValues.Kp = 1.000f;
    cfg.pidYawValues.Ki = 6.00f;
    cfg.pidYawValues.Kd = 0.00040f;
    cfg.pidYawValues.integrationLimit = 10.0f; // Prevent windup
    cfg.pidYawValues.Fc = 100.0f; // 100 Hz

    cfg.pidSonarAltHoldValues.Kp = 0.040f;
    cfg.pidSonarAltHoldValues.Ki = 0.03f;
    cfg.pidSonarAltHoldValues.Kd = 0.00330f;
    cfg.pidSonarAltHoldValues.integrationLimit = 10.0f; // Prevent windup
    cfg.pidSonarAltHoldValues.Fc = 100.0f; // 100 Hz

    cfg.pidBaroAltHoldValues.Kp = 0.006f;
    cfg.pidBaroAltHoldValues.Ki = 0.00f;
    cfg.pidBaroAltHoldValues.Kd = 0.00450f;
    cfg.pidBaroAltHoldValues.integrationLimit = 10.0f; // Prevent windup
    cfg.pidBaroAltHoldValues.Fc = 100.0f; // 100 Hz

    resetPIDRollPitchYaw();
    resetPIDAltHold();

    cfg.angleKp = 4.50f;
    cfg.headKp = 0.65f;
    cfg.maxAngleInclination = 50; // Max angle in self level mode
    cfg.maxAngleInclinationDistSensor = 25; // Max angle when using sonar or LIDAR-Lite v3 in altitude hold mode
    cfg.stickScalingRollPitch = 4.69f;
    cfg.stickScalingYaw = 2.0f;

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
    }
}
