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

/*
pid_t pidRoll, pidPitch, pidYaw; // PID values
float angleKp; // Self level mode Kp value
float stickScalingRollPitch, stickScalingYaw; // Stick scaling values
uint8_t maxAngleInclination; // Max angle in self level mode
*/

struct msg_t {
    uint8_t cmd;
    uint8_t length;
} __attribute__((packed)) msg;

typedef struct {
    uint16_t Kp, Ki, Kd; // PID variables multiplied by 100
    uint16_t integrationLimit; // Integration limit multiplied by 100
} __attribute__((packed)) pidBluetooth_t;

typedef struct {
    uint16_t stickScalingRollPitch, stickScalingYaw; // Stick scaling variables multiplied by 100
} __attribute__((packed)) stickScalingBluetooth_t;

pidBluetooth_t pidRollPitch, pidYaw;
uint16_t angleKp; // Value multiplied by 100
stickScalingBluetooth_t stickScaling; // Stick scaling values
uint8_t maxAngleInclination; // Inclination angle in degrees

#define SET_PID_ROLL_PITCH     0
#define GET_PID_ROLL_PITCH     1
#define SET_PID_YAW            2
#define GET_PID_YAW            3
#define SET_ANGLE_KP           4
#define GET_ANGLE_KP           5
#define SET_STICK_SCALING      6
#define GET_STICK_SCALING      7
#define SET_ANGLE_MAX_INC      8
#define GET_ANGLE_MAX_INC      9
#define SET_KALMAN             10
#define GET_KALMAN             11

struct imu_t {
    int16_t acc, gyro, kalman;
} __attribute__((packed)) imu;

#define START_IMU              12
#define STOP_IMU               13

/*
struct target_t {
  int16_t targetAngle; // Note that this can be negative as well
} __attribute__((packed)) target;

struct kalman_t {
  uint16_t Qangle;
  uint16_t Qbias;
  uint16_t Rmeasure;
} __attribute__((packed)) kalman;

struct info_t {
  uint16_t speed;
  int16_t current; // Note that this can be negative as well
  int16_t turning; // Note that this can be negative as well
  uint16_t battery;
  uint32_t runTime;
} __attribute__((packed)) info;

#define SET_TARGET  2
#define GET_TARGET  3
#define SET_TURNING 4
#define GET_TURNING 5
#define START_INFO  8
#define STOP_INFO   9
*/

const char *commandHeader = "$S>"; // Standard command header
const char *responseHeader = "$S<"; // Standard response header

static bool /*sendSpeed, */sendImu;
static uint32_t /*speedTimer, */imuTimer;

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
                            printPIDValues();
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
                    if (msg.length == 0) {
                        if (getData(NULL, 0)) { // This will check the checksum
#if DEBUG_BLUETOOTH_PROTOCOL
                            UARTprintf("GET_PID_ROLL_PITCH\n");
#endif
                            msg.cmd = GET_PID_ROLL_PITCH;
                            msg.length = sizeof(pidRollPitch);
                            pidRollPitch.Kp = cfg.pidPitch.Kp * 100.0f;
                            pidRollPitch.Ki = cfg.pidPitch.Ki * 100.0f;
                            pidRollPitch.Kd = cfg.pidPitch.Kd * 100.0f;
                            pidRollPitch.integrationLimit = cfg.pidPitch.integrationLimit * 100.0f;
                            sendData((uint8_t*)&pidRollPitch, sizeof(pidRollPitch));
                        }
#if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("GET_PID_ROLL_PITCH checksum error\n");
#endif
                    }
                    break;

#if 0
                case SET_TARGET:
                    if (msg.length == sizeof(target)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&target, sizeof(target))) { // This will read the data and check the checksum
                            cfg.targetAngle = (double)target.targetAngle / 100.0;
                            updateEEPROMValues();
                        }
        #if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            Serial.println(F("SET_TARGET checksum error"));
        #endif
                    }
        #if DEBUG_BLUETOOTH_PROTOCOL
                    else {
                      Serial.print(F("SET_TARGET length error: "));
                      Serial.println(msg.length);
                    }
        #endif
                    break;

                case GET_TARGET:
                    if (msg.length == 0) {
                        if (getData(NULL, 0)) { // This will check the checksum
                            msg.cmd = GET_TARGET;
                            msg.length = sizeof(target);
                            target.targetAngle = cfg.targetAngle * 100.0;
                            sendData((uint8_t*)&target, sizeof(target));
                        }
        #if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            Serial.println(F("GET_PID checksum error"));
        #endif
                    }
                    break;

                case SET_TURNING:
                    if (msg.length == sizeof(turning)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&turning, sizeof(turning))) { // This will read the data and check the checksum
                            cfg.turningScale = turning.turningScale;
                            updateEEPROMValues();
                        }
        #if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            Serial.println(F("SET_TURNING checksum error"));
        #endif
                    }
        #if DEBUG_BLUETOOTH_PROTOCOL
                    else {
                        Serial.print(F("SET_TURNING length error: "));
                        Serial.println(msg.length);
                    }
        #endif
                    break;

                case GET_TURNING:
                    if (msg.length == 0) {
                        if (getData(NULL, 0)) { // This will check the checksum
                            msg.cmd = GET_TURNING;
                            msg.length = sizeof(turning);
                            turning.turningScale = cfg.turningScale;
                            sendData((uint8_t*)&turning, sizeof(turning));
                        }
        #if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            Serial.println(F("GET_TURNING checksum error"));
        #endif
                    }
                    break;

                case SET_KALMAN:
                    if (msg.length == sizeof(kalman)) { // Make sure that it has the right length
                        if (getData((uint8_t*)&kalman, sizeof(kalman))) { // This will read the data and check the checksum
                            cfg.Qangle = kalman.Qangle / 10000.0;
                            cfg.Qbias = kalman.Qbias / 10000.0;
                            cfg.Rmeasure = kalman.Rmeasure / 10000.0;
                            updateEEPROMValues();
                        }
        #if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            Serial.println(F("SET_KALMAN checksum error"));
        #endif
                    }
        #if DEBUG_BLUETOOTH_PROTOCOL
                    else {
                        Serial.print(F("SET_KALMAN length error: "));
                        Serial.println(msg.length);
                    }
        #endif
                    break;

                case GET_KALMAN:
                    if (msg.length == 0) {
                        if (getData(NULL, 0)) { // This will check the checksum
                            msg.cmd = GET_KALMAN;
                            msg.length = sizeof(kalman);
                            kalman.Qangle = cfg.Qangle * 10000.0;
                            kalman.Qbias = cfg.Qbias * 10000.0;
                            kalman.Rmeasure = cfg.Rmeasure * 10000.0;
                            sendData((uint8_t*)&kalman, sizeof(kalman));
                        }
        #if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            Serial.println(F("GET_KALMAN checksum error"));
        #endif
                    }
                    break;

                case START_INFO:
                    if (msg.length == 0) {
                        if (getData(NULL, 0)) // This will check the checksum
                            sendSpeed = true;
        #if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            Serial.println(F("START_INFO checksum error"));
        #endif
                    }
                    break;

                case STOP_INFO:
                    if (msg.length == 0) {
                        if (getData(NULL, 0)) // This will check the checksum
                            sendSpeed = false;
        #if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            Serial.println(F("STOP_INFO checksum error"));
        #endif
                    }
                    break;
#endif
                case START_IMU:
                    if (msg.length == 0) {
                        if (getData(NULL, 0)) // This will read the data and check the checksum
                            sendImu = true;
        #if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("START_IMU checksum error\n");
        #endif
                    }
                    break;

                case STOP_IMU:
                    if (msg.length == 0) {
                        if (getData(NULL, 0)) // This will check the checksum
                            sendImu = false;
        #if DEBUG_BLUETOOTH_PROTOCOL
                        else
                            UARTprintf("STOP_IMU checksum error\n");
        #endif
                    }
                    break;

#if DEBUG_BLUETOOTH_PROTOCOL
                default:
                    UARTprintf("Unknown command: %u\n", msg.cmd);
                    break;
#endif
            }
        }
    }
#if 0
    if (sendSpeed && millis() - speedTimer > 100) {
        speedTimer = millis();
        msg.cmd = START_INFO;
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
        info.runTime = speedTimer;
        sendData((uint8_t*)&info, sizeof(info));
    } else
#endif
    if (sendImu && millis() - imuTimer > 100) {
        imuTimer = millis();
        msg.cmd = START_IMU;
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
// Except the Kalman values which are multiplied by 10000 before sending

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
    return true; // If we get here, then the string have been found
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
    // TODO: Is it really needed to send carriage return and line feed?
    UARTwrite1("\r\n", 2); // Print carriage return and line feed as well, so it is easy to figure out the line ending in Java
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
