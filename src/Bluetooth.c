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
#include <stdlib.h>
#include <string.h>

#include "Bluetooth.h"
#include "EEPROM.h"
#include "Kalman.h"
#include "Time.h"
#include "UART.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor - this is used to print to the terminal
#include "uartstdio1.h" // Add "UART_BUFFERED1" to preprocessor - it uses a modified version of uartstdio, so it can be used with another UART interface

#define DEBUG_BLUETOOTH_PROTOCOL 1

extern float roll, pitch; // Roll and pitch calculated in main.c

enum {
    SET_PID_ROLL_PITCH = 0,
    GET_PID_ROLL_PITCH,
    SET_PID_YAW,
    GET_PID_YAW,
    SET_ANGLE_KP,
    GET_ANGLE_KP,
    SET_STICK_SCALING,
    GET_STICK_SCALING,
    SET_ANGLE_MAX_INC,
    GET_ANGLE_MAX_INC,
    SET_KALMAN,
    GET_KALMAN,
    SEND_IMU,
    SEND_INFO,
};

struct msg_t {
    uint8_t cmd;
    uint8_t length;
} __attribute__((packed)) msg;

typedef struct {
    uint16_t Kp, Ki, Kd; // PID values multiplied by 100
    uint16_t integrationLimit; // Integration limit multiplied by 100
} __attribute__((packed)) pidBluetooth_t;

typedef struct {
    uint16_t stickScalingRollPitch, stickScalingYaw; // Stick scaling values multiplied by 100
} __attribute__((packed)) stickScalingBluetooth_t;

typedef struct {
    uint16_t Q_angle, Q_bias, R_measure; // Kalman coefficients are multiplied by 10000
} __attribute__((packed)) kalmanBluetooth_t;

static pidBluetooth_t pidRollPitch, pidYaw; // PID values multiplied by 100
static uint16_t angleKp; // Value multiplied by 100
static stickScalingBluetooth_t stickScaling; // Stick scaling values multiplied by 100
static uint8_t maxAngleInclination; // Inclination angle in degrees
static kalmanBluetooth_t kalmanCoefficients; // Kalman coefficients are multiplied by 10000
static uint8_t sendInfo, sendImu; // Non-zero if values should be sent

struct imu_t {
    int16_t acc, gyro, kalman;
} __attribute__((packed)) imu;

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

static uint32_t infoTimer, imuTimer;

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

void readBluetoothData() {
    if (UARTRxBytesAvail1()) {
        if (findString(commandHeader)) {
            readBytes((uint8_t*)&msg, sizeof(msg));
            switch (msg.cmd) {
                case SET_PID_ROLL_PITCH:
                    if (msg.length == sizeof(pidRollPitch)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&pidRollPitch, sizeof(pidRollPitch))) { // This will read the data and check the checksum
                            cfg.pidRoll.Kp = cfg.pidPitch.Kp = pidRollPitch.Kp / 100.0f;
                            cfg.pidRoll.Ki = cfg.pidPitch.Ki = pidRollPitch.Ki / 100.0f;
                            cfg.pidRoll.Kd = cfg.pidPitch.Kd = pidRollPitch.Kd / 100.0f;
                            cfg.pidRoll.integrationLimit = cfg.pidPitch.integrationLimit = pidRollPitch.integrationLimit / 100.0f;
                            updateConfig();
#if DEBUG_BLUETOOTH_PROTOCOL
                            printPIDValues(&cfg.pidRoll); // Print PID Values
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
                        msg.length = sizeof(pidRollPitch);
                        pidRollPitch.Kp = cfg.pidPitch.Kp * 100.0f;
                        pidRollPitch.Ki = cfg.pidPitch.Ki * 100.0f;
                        pidRollPitch.Kd = cfg.pidPitch.Kd * 100.0f;
                        pidRollPitch.integrationLimit = cfg.pidPitch.integrationLimit * 100.0f;
                        sendData((uint8_t*)&pidRollPitch, sizeof(pidRollPitch));
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
                    if (msg.length == sizeof(pidYaw)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&pidYaw, sizeof(pidYaw))) { // This will read the data and check the checksum
                            cfg.pidYaw.Kp = pidYaw.Kp / 100.0f;
                            cfg.pidYaw.Ki = pidYaw.Ki / 100.0f;
                            cfg.pidYaw.Kd = pidYaw.Kd / 100.0f;
                            cfg.pidYaw.integrationLimit = pidYaw.integrationLimit / 100.0f;
                            updateConfig();
#if DEBUG_BLUETOOTH_PROTOCOL
                            printPIDValues(&cfg.pidYaw); // Print PID Values
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
                        msg.length = sizeof(pidYaw);
                        pidYaw.Kp = cfg.pidYaw.Kp * 100.0f;
                        pidYaw.Ki = cfg.pidYaw.Ki * 100.0f;
                        pidYaw.Kd = cfg.pidYaw.Kd * 100.0f;
                        pidYaw.integrationLimit = cfg.pidYaw.integrationLimit * 100.0f;
                        sendData((uint8_t*)&pidYaw, sizeof(pidYaw));
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("GET_PID_YAW\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("GET_PID_YAW error\n");
#endif
                    break;

                case SET_ANGLE_KP:
                    if (msg.length == sizeof(angleKp)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&angleKp, sizeof(angleKp))) { // This will read the data and check the checksum
                            cfg.angleKp = angleKp / 100.0f;
                            updateConfig();
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("%d.%02u\n", (int16_t)cfg.angleKp, (uint16_t)(abs(cfg.angleKp * 100.0f) % 100));
                            UARTFlushTx(false);
#endif
                        }
#if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("SET_ANGLE_KP checksum error\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("SET_ANGLE_KP length error: %u\n", msg.length);
#endif
                    break;

                case GET_ANGLE_KP:
                    if (msg.length == 0 && getData(NULL, 0)) { // Check length and the checksum
                        msg.cmd = GET_ANGLE_KP;
                        msg.length = sizeof(angleKp);
                        angleKp = cfg.angleKp * 100.0f;
                        sendData((uint8_t*)&angleKp, sizeof(angleKp));
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("GET_ANGLE_KP\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("GET_ANGLE_KP error\n");
#endif
                    break;
                    
                case SET_STICK_SCALING:
                    if (msg.length == sizeof(stickScaling)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&stickScaling, sizeof(stickScaling))) { // This will read the data and check the checksum
                            cfg.stickScalingRollPitch = stickScaling.stickScalingRollPitch / 100.0f;
                            cfg.stickScalingYaw = stickScaling.stickScalingYaw / 100.0f;
                            updateConfig();
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("%d.%02u\t%d.%02u\n", (int16_t)cfg.stickScalingRollPitch, (uint16_t)(abs(cfg.stickScalingRollPitch * 100.0f) % 100),
                                                             (int16_t)cfg.stickScalingYaw, (uint16_t)(abs(cfg.stickScalingYaw * 100.0f) % 100));
                            UARTFlushTx(false);
#endif
                        }
#if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("SET_STICK_SCALING checksum error\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("SET_STICK_SCALING length error: %u\n", msg.length);
#endif
                    break;

                case GET_STICK_SCALING:
                    if (msg.length == 0 && getData(NULL, 0)) { // Check length and the checksum
                        msg.cmd = GET_STICK_SCALING;
                        msg.length = sizeof(stickScaling);
                        stickScaling.stickScalingRollPitch = cfg.stickScalingRollPitch * 100.0f;
                        stickScaling.stickScalingYaw = cfg.stickScalingYaw * 100.0f;
                        sendData((uint8_t*)&stickScaling, sizeof(stickScaling));
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("GET_STICK_SCALING\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("GET_STICK_SCALING error\n");
#endif
                    break;

                case SET_ANGLE_MAX_INC:
                    if (msg.length == sizeof(maxAngleInclination)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&maxAngleInclination, sizeof(maxAngleInclination))) { // This will read the data and check the checksum
                            cfg.maxAngleInclination = maxAngleInclination;
                            updateConfig();
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("%u\n", cfg.maxAngleInclination);
#endif
                        }
#if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("SET_ANGLE_MAX_INC checksum error\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("SET_ANGLE_MAX_INC length error: %u\n", msg.length);
#endif
                    break;

                case GET_ANGLE_MAX_INC:
                    if (msg.length == 0 && getData(NULL, 0)) { // Check length and the checksum
                        msg.cmd = GET_ANGLE_MAX_INC;
                        msg.length = sizeof(maxAngleInclination);
                        maxAngleInclination = cfg.maxAngleInclination;
                        sendData((uint8_t*)&maxAngleInclination, sizeof(maxAngleInclination));
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("GET_ANGLE_MAX_INC\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("GET_ANGLE_MAX_INC error\n");
#endif
                    break;

                case SET_KALMAN:
                    if (msg.length == sizeof(kalmanCoefficients)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&kalmanCoefficients, sizeof(kalmanCoefficients))) { // This will read the data and check the checksum
                            cfg.Q_angle = kalmanCoefficients.Q_angle / 10000.0f;
                            cfg.Q_bias = kalmanCoefficients.Q_bias / 10000.0f;
                            cfg.R_measure = kalmanCoefficients.R_measure / 10000.0f;
                            updateConfig();
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("Kalman: %d.%04u\t%d.%04u\t%d.%04u\n", (int16_t)cfg.Q_angle, (uint16_t)(abs(cfg.Q_angle * 10000.0f) % 10000),
                                                                              (int16_t)cfg.Q_bias, (uint16_t)(abs(cfg.Q_bias * 10000.0f) % 10000),
                                                                              (int16_t)cfg.R_measure, (uint16_t)(abs(cfg.R_measure * 10000.0f) % 10000));
                            UARTFlushTx(false);
#endif
                        }
#if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("SET_KALMAN checksum error\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("SET_KALMAN length error: %u\n", msg.length);
#endif
                    break;

                case GET_KALMAN:
                    if (msg.length == 0 && getData(NULL, 0)) { // Check length and the checksum
                        msg.cmd = GET_KALMAN;
                        msg.length = sizeof(kalmanCoefficients);
                        kalmanCoefficients.Q_angle = cfg.Q_angle * 10000.0f;
                        kalmanCoefficients.Q_bias = cfg.Q_bias * 10000.0f;
                        kalmanCoefficients.R_measure = cfg.R_measure * 10000.0f;
                        sendData((uint8_t*)&kalmanCoefficients, sizeof(kalmanCoefficients));
#if DEBUG_BLUETOOTH_PROTOCOL
                        UARTprintf("GET_KALMAN\n");
#endif
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("GET_KALMAN error\n");
#endif
                    break;

                case SEND_IMU:
                    if (msg.length == sizeof(sendImu)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&sendImu, sizeof(sendImu))) { // This will read the data and check the checksum
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("sendImu: %u\n", sendImu);
#endif
                        }
                        else {
                            sendImu = 0; // If there was an error, we reset it back to 0, just to be sure
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("SEND_IMU checksum error\n");
#endif
                        }
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("SEND_IMU length error: %u\n", msg.length);
#endif
                    break;

                case SEND_INFO:
                    if (msg.length == sizeof(sendInfo)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&sendInfo, sizeof(sendInfo))) { // This will read the data and check the checksum
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("sendInfo: %u\n", sendInfo);
#endif
                        }
                        else {
                            sendInfo = 0; // If there was an error, we reset it back to 0, just to be sure
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("SEND_INFO checksum error\n");
#endif
                        }
                    }
#if DEBUG_BLUETOOTH_PROTOCOL
                    else
                        UARTprintf("SEND_INFO length error: %u\n", msg.length);
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

    if (sendInfo && millis() - infoTimer > 100) {
#if 0
        infoTimer = millis();
        msg.cmd = SEND_INFO;
        msg.length = sizeof(info);
        info.speed = constrain(abs(PIDValue), 0, 100.0) * 100.0;
        if (deadmanButton::IsSet()) {
            double CS = ((double)analogRead(CS1_PIN) / 204.6 - 2.5) / 0.066 * 100.0; // 66mV/A and then multiply by 100.0
            CS -= ((double)analogRead(CS2_PIN) / 204.6 - 2.5) / 0.066 * 100.0; // The motors turn opposite, so we need to subtract the value
            info.current = CS;
        } else
            info.current = 0; // When the reset button is held low on the motor drivers, the current sensor will give out an incorrect value
        info.turning = turningValue * 100.0;
        info.battery = batteryLevel;
        info.runTime = infoTimer;
        sendData((uint8_t*)&info, sizeof(info));
#endif
    } else if (sendImu && millis() - imuTimer > 100) {
        imuTimer = millis();
        msg.cmd = SEND_IMU;
        msg.length = sizeof(imu);
        imu.acc = 0;
        imu.gyro = roll * 100.0f;
        imu.kalman = pitch * 100.0f;
        sendData((uint8_t*)&imu, sizeof(imu));

        /*UARTprintf("%d\t%d\n", imu.gyro, imu.kalman);
        UARTFlushTx(false);*/
    }
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

// All floats/doubles are multiplied by 100 before sending
// Except the Kalman coefficients which are multiplied by 10000 before sending

static bool findString(const char* string) {
    int pos = UARTPeek1(*string); // Look for the first character
    if (pos == -1) // String was not found
        return false;
    while (pos--)
        UARTgetc1(); // Consume any characters in front

    while (*string != '\0') {
        if (UARTgetc1() != *string++) // Compare with string - note this is a blocking call
            return false;
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
