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
#include <string.h>

#include "Bluetooth.h"
#include "Buzzer.h"
#include "EEPROM.h"
#include "MPU6500.h"
#include "Time.h"
#include "UART.h"
#include "uartstdio1.h" // Add "UART_BUFFERED1" to preprocessor - it uses a modified version of uartstdio, so it can be used with another UART interface

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor - this is used to print to the terminal
#endif

#define DEBUG_BLUETOOTH_PROTOCOL 1 && UART_DEBUG

enum {
    SET_PID_ROLL_PITCH = 0,
    GET_PID_ROLL_PITCH,
    SET_PID_YAW,
    GET_PID_YAW,
    SET_PID_ALT_HOLD,
    GET_PID_ALT_HOLD,
    SET_SETTINGS,
    GET_SETTINGS,
    SEND_ANGLES,
    SEND_INFO, // TODO: Remove
    CAL_ACC,
    //CAL_MAG,
    RESTORE_DEFAULTS,
};

static struct msg_t {
    uint8_t cmd;
    uint8_t length;
} __attribute__((packed)) msg;

typedef struct {
    uint16_t Kp, Ki, Kd; // Kp is multiplied by 1000, Ki multiplied by 100 and Kd are multiplied by 100000
    uint16_t integrationLimit; // Integration limit multiplied by 100
} __attribute__((packed)) pidBT_t;

static struct settings_t {
    uint16_t angleKp, headKp; // Values multiplied by 100
    uint8_t maxAngleInclination; // Inclination angle in degrees
    uint16_t stickScalingRollPitch, stickScalingYaw; // Stick scaling values multiplied by 100
} __attribute__((packed)) settings;

static pidBT_t pidRollPitchBT, pidYawBT, pidAltHoldBT; // PID values
static uint8_t sendAngles; // Non-zero if values should be sent

static struct angles_t {
    int16_t roll, pitch; // Roll and pitch are in the range [-180:180]
    uint16_t yaw; // Yaw is in the range [0:360]
} __attribute__((packed)) angles;

/*
struct info_t {
  uint16_t speed;
  int16_t current; // Note that this can be negative as well
  int16_t turning; // Note that this can be negative as well
  uint16_t battery;
  uint32_t runTime;
} __attribute__((packed)) info;
*/

static const char *commandHeader = "$S>"; // Standard command header
static const char *responseHeader = "$S<"; // Standard response header

static bool findString(const char* string);
static void readBytes(uint8_t* data, uint8_t length);
static bool getData(uint8_t *data, uint8_t length);
static void sendData(uint8_t *data, uint8_t length);
static uint8_t getCheckSum(uint8_t *data, size_t length);

void initBluetooth(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable the GPIO port containing the pins that will be used.
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    // Configure the GPIO pin muxing for the UART function.
    // This is only necessary if your part supports GPIO pin function muxing.
    // Study the data sheet to see which functions are allocated per pin.
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);

    // Since GPIO B0 and B1 are used for the UART function, they must be
    // configured for use as a peripheral function (instead of GPIO).
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTStdioConfig1(1, 115200, SysCtlClockGet()); // Mode is set to 8N1 on UART1
    UARTEchoSet1(false);

    while (UARTBusy(UART1_BASE)) {
        // Wait until UART is ready
    }
}

bool readBluetoothData(mpu6500_t *mpu6500, angle_t *angle) {
    bool newValuesReceived = false;
    if (UARTRxBytesAvail1() > strlen(commandHeader)) {
        if (findString(commandHeader)) {
            readBytes((uint8_t*)&msg, sizeof(msg));
            switch (msg.cmd) {
                case SET_PID_ROLL_PITCH:
                    if (msg.length == sizeof(pidRollPitchBT)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&pidRollPitchBT, sizeof(pidRollPitchBT))) { // This will read the data and check the checksum
                            pidRoll.values->Kp = pidPitch.values->Kp = pidRollPitchBT.Kp / 1000.0f;
                            pidRoll.values->Ki = pidPitch.values->Ki = pidRollPitchBT.Ki / 100.0f;
                            pidRoll.values->Kd = pidPitch.values->Kd = pidRollPitchBT.Kd / 100000.0f;
                            pidRoll.values->integrationLimit = pidPitch.values->integrationLimit = pidRollPitchBT.integrationLimit / 100.0f;
                            updateConfig();
                            newValuesReceived = true;
#if DEBUG_BLUETOOTH_PROTOCOL
                            printPIDValues(pidRoll.values); // Print PID Values
#endif
                        }
#if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("SET_PID_ROLL_PITCH checksum error\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("SET_PID_ROLL_PITCH length error: %u\n", msg.length);
#endif
                    break;

                case GET_PID_ROLL_PITCH:
                    if (msg.length == 0 && getData(NULL, 0)) { // Check length and the checksum
                        msg.cmd = GET_PID_ROLL_PITCH;
                        msg.length = sizeof(pidRollPitchBT);
                        pidRollPitchBT.Kp = pidPitch.values->Kp * 1000.0f;
                        pidRollPitchBT.Ki = pidPitch.values->Ki * 100.0f;
                        pidRollPitchBT.Kd = pidPitch.values->Kd * 100000.0f;
                        pidRollPitchBT.integrationLimit = pidPitch.values->integrationLimit * 100.0f;
                        sendData((uint8_t*)&pidRollPitchBT, sizeof(pidRollPitchBT));
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("GET_PID_ROLL_PITCH\n");
#endif
                     }
#if DEBUG_BLUETOOTH_PROTOCOL
                     else
                        UARTprintf("GET_PID_ROLL_PITCH error\n");
#endif
                    break;

                case SET_PID_YAW:
                    if (msg.length == sizeof(pidYawBT)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&pidYawBT, sizeof(pidYawBT))) { // This will read the data and check the checksum
                            pidYaw.values->Kp = pidYawBT.Kp / 1000.0f;
                            pidYaw.values->Ki = pidYawBT.Ki / 100.0f;
                            pidYaw.values->Kd = pidYawBT.Kd / 100000.0f;
                            pidYaw.values->integrationLimit = pidYawBT.integrationLimit / 100.0f;
                            updateConfig();
                            newValuesReceived = true;
#if DEBUG_BLUETOOTH_PROTOCOL
                            printPIDValues(pidYaw.values); // Print PID Values
#endif
                        }
#if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("SET_PID_YAW checksum error\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("SET_PID_YAW length error: %u\n", msg.length);
#endif
                    break;

                case GET_PID_YAW:
                    if (msg.length == 0 && getData(NULL, 0)) { // Check length and the checksum
                        msg.cmd = GET_PID_YAW;
                        msg.length = sizeof(pidYawBT);
                        pidYawBT.Kp = pidYaw.values->Kp * 1000.0f;
                        pidYawBT.Ki = pidYaw.values->Ki * 100.0f;
                        pidYawBT.Kd = pidYaw.values->Kd * 100000.0f;
                        pidYawBT.integrationLimit = pidYaw.values->integrationLimit * 100.0f;
                        sendData((uint8_t*)&pidYawBT, sizeof(pidYawBT));
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("GET_PID_YAW\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("GET_PID_YAW error\n");
#endif
                    break;

                case SET_PID_ALT_HOLD:
                    if (msg.length == sizeof(pidAltHoldBT)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&pidAltHoldBT, sizeof(pidAltHoldBT))) { // This will read the data and check the checksum
                            pidAltHold.values->Kp = pidAltHoldBT.Kp / 1000.0f;
                            pidAltHold.values->Ki = pidAltHoldBT.Ki / 100.0f;
                            pidAltHold.values->Kd = pidAltHoldBT.Kd / 100000.0f;
                            pidAltHold.values->integrationLimit = pidAltHoldBT.integrationLimit / 100.0f;
                            updateConfig();
                            newValuesReceived = true;
#if DEBUG_BLUETOOTH_PROTOCOL
                            printPIDValues(pidAltHold.values); // Print PID Values
#endif
                        }
#if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("SET_PID_ALT_HOLD checksum error\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("SET_PID_ALT_HOLD length error: %u\n", msg.length);
#endif
                    break;

                case GET_PID_ALT_HOLD:
                    if (msg.length == 0 && getData(NULL, 0)) { // Check length and the checksum
                        msg.cmd = GET_PID_ALT_HOLD;
                        msg.length = sizeof(pidAltHoldBT);
                        pidAltHoldBT.Kp = pidAltHold.values->Kp * 1000.0f;
                        pidAltHoldBT.Ki = pidAltHold.values->Ki * 100.0f;
                        pidAltHoldBT.Kd = pidAltHold.values->Kd * 100000.0f;
                        pidAltHoldBT.integrationLimit = pidAltHold.values->integrationLimit * 100.0f;
                        sendData((uint8_t*)&pidAltHoldBT, sizeof(pidAltHoldBT));
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("GET_PID_ALT_HOLD\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("GET_PID_ALT_HOLD error\n");
#endif
                    break;

                case SET_SETTINGS:
                    if (msg.length == sizeof(settings)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&settings, sizeof(settings))) { // This will read the data and check the checksum
                            cfg.angleKp = settings.angleKp / 100.0f;
                            cfg.headKp = settings.headKp / 100.0f;
                            cfg.maxAngleInclination = settings.maxAngleInclination;
                            cfg.stickScalingRollPitch = settings.stickScalingRollPitch / 100.0f;
                            cfg.stickScalingYaw = settings.stickScalingYaw / 100.0f;
                            updateConfig();
                            newValuesReceived = true;
#if DEBUG_BLUETOOTH_PROTOCOL
                            printSettings();
#endif
                        }
#if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("SET_SETTINGS checksum error\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("SET_SETTINGS length error: %u\n", msg.length);
#endif
                    break;

                case GET_SETTINGS:
                    if (msg.length == 0 && getData(NULL, 0)) { // Check length and the checksum
                        msg.cmd = GET_SETTINGS;
                        msg.length = sizeof(settings);
                        settings.angleKp = cfg.angleKp * 100.0f;
                        settings.headKp = cfg.headKp * 100.0f;
                        settings.maxAngleInclination = cfg.maxAngleInclination;
                        settings.stickScalingRollPitch = cfg.stickScalingRollPitch * 100.0f;
                        settings.stickScalingYaw = cfg.stickScalingYaw * 100.0f;
                        sendData((uint8_t*)&settings, sizeof(settings));
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("GET_SETTINGS\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("GET_SETTINGS error\n");
#endif
                    break;

                case SEND_ANGLES:
                    if (msg.length == sizeof(sendAngles)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&sendAngles, sizeof(sendAngles))) { // This will read the data and check the checksum
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("sendAngles: %u\n", sendAngles);
#endif
                        }
                        else {
                            sendAngles = 0; // If there was an error, we reset it back to 0, just to be sure
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("SEND_ANGLES checksum error\n");
#endif
                        }
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("SEND_IMU length error: %u\n", msg.length);
#endif
                    break;

                case CAL_ACC:
                    if (msg.length == 0 && getData(NULL, 0)) { // Check length and the checksum
                        while (calibrateAcc(mpu6500)) { // Get accelerometer zero values
                            // Loop until calibration values are found
                        }
                        beepLongBuzzer();
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("CAL_ACC\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("CAL_ACC error\n");
#endif
                    break;

                 case RESTORE_DEFAULTS:
                    if (msg.length == 0 && getData(NULL, 0)) { // Check length and the checksum
                        setDefaultConfig();
                        beepLongBuzzer();
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("RESTORE_DEFAULTS\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("RESTORE_DEFAULTS error\n");
#endif
                    break;

#if DEBUG_BLUETOOTH_PROTOCOL
                default:
                    UARTprintf("Unknown command: %u\n", msg.cmd);
                    break;
#endif
            }
        }
    }

    static uint32_t angleTimer = 0;
    if (sendAngles && (int32_t)(millis() - angleTimer) > 10) {
        angleTimer = millis();
        msg.cmd = SEND_ANGLES;
        msg.length = sizeof(angles);
        angles.roll = angle->axis.roll * 100.0f;
        angles.pitch = angle->axis.pitch * 100.0f;
        angles.yaw = angle->axis.yaw * 100.0f;
        sendData((uint8_t*)&angles, sizeof(angles));

#if 0 && DEBUG_BLUETOOTH_PROTOCOL
        UARTprintf("%d\t%d\t%u\n", angles.roll, angles.pitch, angles.yaw);
        UARTFlushTx(false);
#endif
    }

    return newValuesReceived;
}

// Message protocol (Inspired by MultiWii):
// Request:
// Header: $S>
// cmd: uint8_t
// n length of data: uint8_t
// Data: n uint8_t
// Checksum (calculated from cmd, length and data)

// Response:
// Header: $S<
// cmd: uint8_t
// n length of data: uint8_t
// Data: n uint8_t
// Checksum (calculated from cmd, length and data)
// Carriage return and line feed ("\r\n")

static bool findString(const char* string) {
    int nbytes = UARTRxBytesAvail1();
    int pos = UARTPeek1(*string); // Look for the first character
    if (pos == -1) { // String was not found
        while (nbytes--)
            UARTgetc1(); // Consume all characters in the buffer
#if DEBUG_BLUETOOTH_PROTOCOL
        UARTprintf("Could not find string\n");
#endif
        return false;
    }
    while (pos--)
        UARTgetc1(); // Consume any characters in front

    while (*string != '\0') {
        if (UARTgetc1() != *string++) { // Compare with string - note this is a blocking call
#if DEBUG_BLUETOOTH_PROTOCOL
            UARTprintf("String mismatch\n");
#endif
            return false;
        }
    }
    return true; // If we get here, then the string has been found
}

static void readBytes(uint8_t* data, uint8_t length) {
    for (uint8_t i = 0; i < length; i++)
        data[i] = UARTgetc1(); // Store data in buffer - note this is a blocking call
}

static bool getData(uint8_t *data, uint8_t length) {
    if (length > 0)
        readBytes(data, length); // Read data into buffer
    uint8_t checksum;
    readBytes(&checksum, sizeof(checksum)); // Read the checksum
    return (getCheckSum((uint8_t*)&msg, sizeof(msg)) ^ getCheckSum(data, length)) == checksum; // The checksum is calculated from the length, command and the data
}

static void sendData(uint8_t *data, uint8_t length) {
    const char checksum = getCheckSum((uint8_t*)&msg, sizeof(msg)) ^ getCheckSum(data, length);

    UARTwrite1(responseHeader, strlen(responseHeader));
    UARTwrite1((const char*)&msg, sizeof(msg));
    UARTwrite1((const char*)data, length);
    UARTwrite1(&checksum, sizeof(checksum)); // The checksum is calculated from the length, command and the data
    UARTwrite1("\r\n", 2); // Print carriage return and line feed as well, this is needed for the Android application
    UARTFlushTx1(false); // Flush TX buffer
}

static uint8_t getCheckSum(uint8_t *data, size_t length) {
    if (length == 0)
        return 0;
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++)
        checksum ^= data[i]; // Calculate checksum
    return checksum;
}
