/* Copyright (C) 2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

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

#include "MPU6500.h"
#include "time.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

#include "math.h"

#define SLAVE_ADDRESS	0x68
#define RAD_TO_DEG 		57.295779513082320876798154814105f

void initMPU6500_i2c(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1); // Enable I2C1 peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIOA peripheral

	// Use altenate function
	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);

	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6); // Use pin with I2C SCL peripheral
	GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7); // Use pin with I2C peripheral

	I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true); // Enable and set frequency to 400 kHz

	uint8_t i2cBuffer[14]; // Buffer for I2C data

	i2cBuffer[0] = i2cRead(0x75);
	if (i2cBuffer[0] == 0x70) // Read "WHO_AM_I" register
		UARTprintf("MPU-6500 found\n");
	else {
		UARTprintf("Could not find MPU-6500: %2X\n", i2cBuffer[0]);
		while (1);
	}

	i2cWrite(0x6B, (1 << 7)); // Reset device, this resets all internal registers to their default values
	delay(100);
	while (i2cRead(0x6B) & (1 << 7)) {
		// Wait for the bit to clear
	};
	delay(100);
	i2cWrite(0x6B, (1 << 3) | (1 << 0)); // Disable sleep mode, disable temperature sensor and use PLL as clock reference

	i2cBuffer[0] = 0; // Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
	i2cBuffer[1] = 0x03; // Disable FSYNC and set 44 Hz Acc filtering, 42 Hz Gyro filtering, 1 KHz sampling
	i2cBuffer[2] = 3 << 3; // Set Gyro Full Scale Range to ±2000deg/s
	i2cBuffer[3] = 2 << 3; // Set Accelerometer Full Scale Range to ±8g
	i2cWriteData(0x19, i2cBuffer, 4); // Write to all four registers at once

#if 0
	/* Enable Data Ready Interrupt on INT pin */
	i2cBuffer[0] = (1 << 5) | (1 << 4); // Enable LATCH_INT_EN and INT_RD_CLEAR
	                                  // When this bit is equal to 1, the INT pin is held high until the interrupt is cleared
	                                  // When this bit is equal to 1, interrupt status bits are cleared on any read operation
	i2cBuffer[1] = (1 << 0); // Enable DATA_RDY_EN - When set to 1, this bit enables the Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed
	i2cWriteData(0x37, i2cBuffer, 2); // Write to both registers at once
#endif

	delay(100); // Wait for sensor to stabilize

	float gyroRate[3];
	static float gyroAngle[3] = { 0, 0, 0 };
	int16_t accData[3], gyroData[3], gyroZero[3];

	updateMPU6500(accData, gyroZero);

	delay(100);

	while (1) {
		updateMPU6500(accData, gyroData);

		static uint32_t timer = 0;
		float dt = (float)(micros() - timer) / 1000000.0f;
		timer = micros();

		for (uint8_t axis = 0; axis < 3; axis++) {
			gyroRate[axis] = (float)(gyroData[axis] - gyroZero[axis]) / 16.4f;
			gyroAngle[axis] += gyroRate[axis] * dt; // Gyro angle is only used for debugging
		}

		float roll  = atan2f(accData[1], accData[2]) * RAD_TO_DEG;
		float pitch = atanf(-accData[0] / sqrt(accData[1] * accData[1] + accData[2] * accData[2])) * RAD_TO_DEG;

		static float compAngleX, compAngleY;

		compAngleX = 0.93f * (compAngleX + gyroRate[0] * dt) + 0.07f * roll; // Calculate the angle using a Complimentary filter
		compAngleY = 0.93f * (compAngleY + gyroRate[1] * dt) + 0.07f * pitch;

		UARTprintf("%d\t%d\t%d\t%d\t%d\t%d\t%d\n", (int16_t)compAngleX, (int16_t)compAngleY, (int16_t)roll, (int16_t)pitch, (int16_t)gyroAngle[0], (int16_t)gyroAngle[1], (int16_t)gyroAngle[2]);

		delay(10);
	}
}

void updateMPU6500(int16_t *accData, int16_t *gyroData) {
	uint8_t buf[14];

	i2cReadData(0x3B, buf, 14);

	accData[0] = (buf[0] << 8) | buf[1];
	accData[1] = (buf[2] << 8) | buf[3];
	accData[2] = (buf[4] << 8) | buf[5];

	gyroData[0] = (buf[8] << 8) | buf[9];
	gyroData[1] = (buf[10] << 8) | buf[11];
	gyroData[2] = (buf[12] << 8) | buf[13];
}

void i2cWrite(uint8_t addr, uint8_t data) {
	i2cWriteData(addr, &data, 1);
}

void i2cWriteData(uint8_t addr, uint8_t *data, uint8_t length) {
	I2CMasterSlaveAddrSet(I2C1_BASE, SLAVE_ADDRESS, false); // Set to write mode

	I2CMasterDataPut(I2C1_BASE, addr); // Place addreess into data register
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send start condition
	while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done

	for (uint8_t i = 0; i < length - 1; i++) {
		I2CMasterDataPut(I2C1_BASE, data[i]); // Place data into data register
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT); // Send continues condition
		while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
	}

	I2CMasterDataPut(I2C1_BASE, data[length - 1]); // Place data into data register
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // Send finish condition
	while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
}

uint8_t i2cRead(uint8_t addr) {
	I2CMasterSlaveAddrSet(I2C1_BASE, SLAVE_ADDRESS, false); // Set to write mode

	I2CMasterDataPut(I2C1_BASE, addr); // Place addreess into data register
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND); // Send data
	while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done

	I2CMasterSlaveAddrSet(I2C1_BASE, SLAVE_ADDRESS, true); // Set to read mode

	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); // Tell master to read data
	while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
	return I2CMasterDataGet(I2C1_BASE); // Read data
}

void i2cReadData(uint8_t addr, uint8_t *data, uint8_t length) {
	I2CMasterSlaveAddrSet(I2C1_BASE, SLAVE_ADDRESS, false); // Set to write mode

	I2CMasterDataPut(I2C1_BASE, addr); // Place addreess into data register
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND); // Send data
	while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done

	I2CMasterSlaveAddrSet(I2C1_BASE, SLAVE_ADDRESS, true); // Set to read mode

	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); // Send start condition
	while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
	data[0] = I2CMasterDataGet(I2C1_BASE); // Place data into data register

	for (uint8_t i = 1; i < length - 1; i++) {
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); // Send continues condition
		while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
		data[i] = I2CMasterDataGet(I2C1_BASE); // Place data into data register
	}

	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); // Send finish condition
	while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
	data[length - 1] = I2CMasterDataGet(I2C1_BASE); // Place data into data register
}

void initMPU6500(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0); // Enable SSI0 peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIOA peripheral

	// Use altenate function
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5); // Use pin with SSI peripheral

	//SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM); // Set clock source

	// Configure the SSI to MODE0, 1 MHz, and 8-bit data
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);
	SSIEnable(SSI0_BASE); // Enable the SSI module

	// PWR_MGMT_1
	// Reset the device
	spiWriteData(0x6B, 0x80);

	delay(100);

	// PWR_MGMT_1
	// Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
	// Disable sleep mode
	spiWriteData(0x6B, 0x01);

	delay(100);

	// USER_CTRL
	// Reset I2C Slave module and put the serial interface in SPI mode only
	spiWriteData(0x6A, 0x10/* | 0x8 | 0x4 | 0x1*/);

	delay(100);

/*
  uint32_t pui32DataRx[3];
	while (SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0]))
  {
		// Empty FIFO
  }
*/
	uint32_t buffer[100];
	spiReadData(0x75, buffer);

	if (buffer[0] == 0x70)
		UARTprintf("MPU-6500 initialized\n");
	else
		UARTprintf("Could not initialize MPU-6500: %2X\n", buffer[0]);
}

void spiReadData(uint32_t addr, uint32_t *buffer) {
	SSIDataPut(SSI0_BASE, addr | 0x80); // Indicate read operation
	while (SSIBusy(SSI0_BASE));
	SSIDataGet(SSI0_BASE, buffer);
	while (SSIBusy(SSI0_BASE));
}

void spiWriteData(uint32_t addr, uint32_t buffer) {
	SSIDataPut(SSI0_BASE, addr);
	while (SSIBusy(SSI0_BASE));
	SSIDataPut(SSI0_BASE, buffer);
	while (SSIBusy(SSI0_BASE));
}
