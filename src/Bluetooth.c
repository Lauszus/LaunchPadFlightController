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
#include <string.h>

#include "Bluetooth.h"
#include "Buzzer.h"
#include "EEPROM.h"
#include "Magnetometer.h"
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

#define DEBUG_BLUETOOTH_PROTOCOL 0 && UART_DEBUG

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
    CAL_ACC,
    CAL_MAG,
    RESTORE_DEFAULTS,
};

typedef struct {
    uint8_t cmd;
    uint8_t length;
} __attribute__((packed)) msg_t;

typedef struct {
    uint16_t Kp, Ki, Kd; // Kp is multiplied by 1000, Ki multiplied by 100 and Kd are multiplied by 100000
    uint16_t integrationLimit; // Integration limit multiplied by 100
} __attribute__((packed)) pid_values_bt_t;

typedef struct {
    uint16_t angleKp, headKp; // Values multiplied by 100
    uint8_t maxAngleInclination, maxAngleInclinationSonar; // Inclination angle in degrees
    uint16_t stickScalingRollPitch, stickScalingYaw; // Stick scaling values multiplied by 100
} __attribute__((packed)) settings_t;

static uint8_t sendAngles; // Non-zero if values should be sent

typedef struct {
    int16_t roll, pitch; // Roll and pitch are in the range [-180:180]
    uint16_t yaw; // Yaw is in the range [0:360]
} __attribute__((packed)) angles_t;

static const char *commandHeader = "$S>"; // Standard command header
static const char *responseHeader = "$S<"; // Standard response header

static bool findString(const char *string);
static void readBytes(uint8_t *data, size_t length);
static bool getData(msg_t msg, uint8_t *data);
static void sendData(msg_t msg, uint8_t *data);
static uint8_t getCheckSum(uint8_t *data, size_t length);
static pid_values_t* getPidValuesPointer(uint8_t cmd);

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
    msg_t msg;

    if (UARTRxBytesAvail1() > strlen(commandHeader)) {
        if (findString(commandHeader)) {
            readBytes((uint8_t*)&msg, sizeof(msg));
            switch (msg.cmd) {
                case SET_PID_ROLL_PITCH:
                case SET_PID_YAW:
                case SET_PID_ALT_HOLD:
                    if (msg.length == sizeof(pid_values_bt_t)) { // Make sure that it has the right length
                        pid_values_t *pidValues = getPidValuesPointer(msg.cmd);
                        if (!pidValues)
                            break; // Abort

                        pid_values_bt_t pidValuesBt;
                        if (getData(msg, (uint8_t*)&pidValuesBt)) { // This will read the data and check the checksum
                            pidValues->Kp = pidValuesBt.Kp / 1000.0f;
                            pidValues->Ki = pidValuesBt.Ki / 100.0f;
                            pidValues->Kd = pidValuesBt.Kd / 100000.0f;
                            pidValues->integrationLimit = pidValuesBt.integrationLimit / 100.0f;
                            updateConfig();
                            newValuesReceived = true;
#if DEBUG_BLUETOOTH_PROTOCOL
                            printPIDValues(pidValues); // Print PID Values
#endif
                        }
#if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("Set PID checksum error\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("Set PID length error: %u\n", msg.length);
#endif
                    break;

                case GET_PID_ROLL_PITCH:
                case GET_PID_YAW:
                case GET_PID_ALT_HOLD:
                    if (msg.length == 0 && getData(msg, NULL)) { // Check length and the checksum
                        pid_values_t *pidValues = getPidValuesPointer(msg.cmd);
                        if (!pidValues)
                            break; // Abort

                        msg.length = sizeof(pid_values_bt_t);
                        pid_values_bt_t pidValuesBt;
                        pidValuesBt.Kp = pidValues->Kp * 1000.0f;
                        pidValuesBt.Ki = pidValues->Ki * 100.0f;
                        pidValuesBt.Kd = pidValues->Kd * 100000.0f;
                        pidValuesBt.integrationLimit = pidValues->integrationLimit * 100.0f;
                        sendData(msg, (uint8_t*)&pidValuesBt);
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("Get PID %u\n", msg.cmd);
#endif
                     }
#if DEBUG_BLUETOOTH_PROTOCOL
                     else
                        UARTprintf("Get PID error\n");
#endif
                    break;

                case SET_SETTINGS:
                    if (msg.length == sizeof(settings_t)) { // Make sure that it has the right length
                        settings_t settings;
                        if (getData(msg, (uint8_t*)&settings)) { // This will read the data and check the checksum
                            cfg.angleKp = settings.angleKp / 100.0f;
                            cfg.headKp = settings.headKp / 100.0f;
                            cfg.maxAngleInclination = settings.maxAngleInclination;
                            cfg.maxAngleInclinationSonar = settings.maxAngleInclinationSonar;
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
                    if (msg.length == 0 && getData(msg, NULL)) { // Check length and the checksum
                        msg.length = sizeof(settings_t);
                        settings_t settings;
                        settings.angleKp = cfg.angleKp * 100.0f;
                        settings.headKp = cfg.headKp * 100.0f;
                        settings.maxAngleInclination = cfg.maxAngleInclination;
                        settings.maxAngleInclinationSonar = cfg.maxAngleInclinationSonar;
                        settings.stickScalingRollPitch = cfg.stickScalingRollPitch * 100.0f;
                        settings.stickScalingYaw = cfg.stickScalingYaw * 100.0f;
                        sendData(msg, (uint8_t*)&settings);
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
                        if (getData(msg, (uint8_t*)&sendAngles)) { // This will read the data and check the checksum
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("sendAngles: %u\n", sendAngles);
#endif
                        } else {
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
                    if (msg.length == 0 && getData(msg, NULL)) { // Check length and the checksum
                        while (calibrateMPU6500Acc(mpu6500)) { // Get accelerometer zero values
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

                case CAL_MAG:
#if USE_MAG
                    if (msg.length == 0 && getData(msg, NULL)) { // Check length and the checksum
                        calibrateMag(); // Get magnetometer zero values
                        beepLongBuzzer();
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("CAL_MAG\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("CAL_MAG error\n");
#endif
#endif // USE_MAG
                    break;

                 case RESTORE_DEFAULTS:
                    if (msg.length == 0 && getData(msg, NULL)) { // Check length and the checksum
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
        msg.length = sizeof(angles_t);
        angles_t angles;
        angles.roll = angle->axis.roll * 100.0f;
        angles.pitch = angle->axis.pitch * 100.0f;
        angles.yaw = angle->axis.yaw * 100.0f;
        sendData(msg, (uint8_t*)&angles);

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

static bool findString(const char *string) {
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

static void readBytes(uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++)
        data[i] = UARTgetc1(); // Store data in buffer - note this is a blocking call
}

static bool getData(msg_t msg, uint8_t *data) {
    if (msg.length > 0)
        readBytes(data, msg.length); // Read data into buffer
    uint8_t checksum = UARTgetc1(); // Read the checksum - note this is a blocking call
    return (getCheckSum((uint8_t*)&msg, sizeof(msg)) ^ getCheckSum(data, msg.length)) == checksum; // The checksum is calculated from the length, command and the data
}

static void sendData(msg_t msg, uint8_t *data) {
    const char checksum = getCheckSum((uint8_t*)&msg, sizeof(msg)) ^ getCheckSum(data, msg.length);

    UARTwrite1(responseHeader, strlen(responseHeader));
    UARTwrite1((const char*)&msg, sizeof(msg));
    UARTwrite1((const char*)data, msg.length);
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

static pid_values_t* getPidValuesPointer(uint8_t cmd) {
    if (cmd == SET_PID_ROLL_PITCH || cmd == GET_PID_ROLL_PITCH)
        return &cfg.pidRollPitchValues;
    else if (cmd == SET_PID_YAW || cmd == GET_PID_YAW)
        return &cfg.pidYawValues;
    else if (cmd == SET_PID_ALT_HOLD || cmd == GET_PID_ALT_HOLD)
        return &cfg.pidAltHoldValues;

    return NULL;
}
