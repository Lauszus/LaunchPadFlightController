/* Copyright (C) 2017 Kristian Sloth Lauszus. All rights reserved.

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

#ifndef __config_h__
#define __config_h__

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#ifdef __cplusplus
extern "C" {
#endif

// LEDs used in various places
#define SYSCTL_PERIPH_LED               SYSCTL_PERIPH_GPIOF
#define GPIO_LED_BASE                   GPIO_PORTF_BASE
#define GPIO_RED_LED                    GPIO_PIN_1
#define GPIO_BLUE_LED                   GPIO_PIN_2
#define GPIO_GREEN_LED                  GPIO_PIN_3

// ADNS3080.c
#define GPIO_ADNS3080_RESET_PERIPH      SYSCTL_PERIPH_GPIOE
#define GPIO_ADNS3080_RESET_BASE        GPIO_PORTE_BASE
#define GPIO_ADNS3080_RESET_PIN         GPIO_PIN_1

// Bluetooth.c
#define SYSCTL_PERIPH_BLUETOOTH         SYSCTL_PERIPH_GPIOB
#define GPIO_RX_BLUETOOTH               GPIO_PB0_U1RX
#define GPIO_TX_BLUETOOTH               GPIO_PB1_U1TX
#define GPIO_BLUETOOTH_PIN_BASE         GPIO_PORTB_BASE
#define GPIO_RX_PIN_BLUETOOTH           GPIO_PIN_0
#define GPIO_TX_PIN_BLUETOOTH           GPIO_PIN_1
#define UART_NR_BLUETOOTH               1
#define UART_BASE_BLUETOOTH             UART1_BASE

// Buzzer.c
#define SYSCTL_PERIPH_BUZZER            SYSCTL_PERIPH_GPIOD
#define GPIO_BUZZER_BASE                GPIO_PORTD_BASE
#define GPIO_PIN_BUZZER                 GPIO_PIN_2

// HMC5883L.c
#define GPIO_HMC5883L_DRDY_PERIPH       SYSCTL_PERIPH_GPIOE
#define GPIO_HMC5883L_DRDY_BASE         GPIO_PORTE_BASE
#define GPIO_HMC5883L_DRDY_PIN          GPIO_PIN_3

// MPU6500.c
#define GPIO_MPU_INT_PERIPH             SYSCTL_PERIPH_GPIOE
#define GPIO_MPU_INT_BASE               GPIO_PORTE_BASE
#define GPIO_MPU_INT_PIN                GPIO_PIN_2

// PPM.c
#define SYSCTL_PERIPH_SW                SYSCTL_PERIPH_GPIOF
#define GPIO_SW_BASE                    GPIO_PORTF_BASE
#define GPIO_SW1                        GPIO_PIN_4
#define GPIO_SW2                        GPIO_PIN_0

// RX.c
// These are specific to my receiver and might need adjustment
#define RX_MIN_INPUT                    980
#define RX_MAX_INPUT                    2032

#define SYSCTL_PERIPH_RX                SYSCTL_PERIPH_GPIOC
#define GPIO_RX_BASE                    GPIO_PORTC_BASE
#define GPIO_RX                         GPIO_PIN_6

#define SYSCTL_PERIPH_RX_TIMER          SYSCTL_PERIPH_WTIMER1
#define GPIO_RX_ALTERNATE               GPIO_PC6_WT1CCP0
#define RX_TIMER_BASE                   WTIMER1_BASE
#define RX_TIMER_INT                    INT_WTIMER1A
#define RX_TIMER                        TIMER_A
#define RX_INT_FLAG                     TIMER_CAPA_EVENT
#define RX_TIMER_CFG                    TIMER_CFG_A_CAP_TIME_UP

#define SYSCTL_PERIPH_RX_LOST_TIMER     SYSCTL_PERIPH_WTIMER1
#define RX_LOST_TIMER_BASE              WTIMER1_BASE
#define RX_LOST_TIMER_INT               INT_WTIMER1B
#define RX_LOST_TIMER                   TIMER_B
#define RX_LOST_INT_FLAG                TIMER_TIMB_TIMEOUT
#define RX_LOST_TIMER_CFG               TIMER_CFG_B_PERIODIC

// Sonar.c
#define SYSCTL_PERIPH_TRIG              SYSCTL_PERIPH_GPIOE
#define GPIO_SONAR_TRIG_BASE            GPIO_PORTE_BASE
#define GPIO_SONAR_TRIG                 GPIO_PIN_0

#define SYSCTL_PERIPH_ECHO              SYSCTL_PERIPH_GPIOC
#define GPIO_SONAR_ECHO_BASE            GPIO_PORTC_BASE
#define GPIO_SONAR_ECHO                 GPIO_PIN_5

// Timer used to measure the width of the sonar echo pulse
#define SYSCTL_PERIPH_SONAR_TIMER       SYSCTL_PERIPH_WTIMER0
#define GPIO_SONAR_ALTERNATE            GPIO_PC5_WT0CCP1
#define SONAR_TIMER_BASE                WTIMER0_BASE
#define SONAR_TIMER_INT                 INT_WTIMER0B
#define SONAR_TIMER                     TIMER_B
#define SONAR_CAP_EVENT                 TIMER_CAPB_EVENT
#define SONAR_TIMER_CFG                 TIMER_CFG_B_CAP_TIME_UP

// startup_gcc.c
#define UART0_HANDLER                   UARTStdioIntHandler
#define UART1_HANDLER                   UARTStdioIntHandler1
#define UART2_HANDLER                   IntDefaultHandler
#define UART3_HANDLER                   IntDefaultHandler
#define UART4_HANDLER                   IntDefaultHandler
#define UART5_HANDLER                   IntDefaultHandler
#define UART6_HANDLER                   IntDefaultHandler
#define UART7_HANDLER                   IntDefaultHandler

// UART.c
#define SYSCTL_PERIPH_UART              SYSCTL_PERIPH_GPIOA
#define GPIO_RX_UART                    GPIO_PA0_U0RX
#define GPIO_TX_UART                    GPIO_PA1_U0TX
#define GPIO_UART_PIN_BASE              GPIO_PORTA_BASE
#define GPIO_RX_PIN_UART                GPIO_PIN_0
#define GPIO_TX_PIN_UART                GPIO_PIN_1
#define UART_NR_UART                    0
#define UART_BASE_UART                  UART0_BASE

#ifdef __cplusplus
}
#endif

// Check if "Config_custom.h" exist
// This is used to redefine the default values above
#ifdef __has_include
#if __has_include("Config_custom.h")
#include "Config_custom.h"
#endif
#endif

#endif
