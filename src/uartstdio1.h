//*****************************************************************************
//
// uartstdio.h - Prototypes for the UART console functions.
//
// Copyright (c) 2007-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 1.1 of the Tiva Utility Library.
//
//*****************************************************************************

#ifndef __UARTSTDIO1_H__
#define __UARTSTDIO1_H__

#include <stdarg.h>

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// If built for buffered operation, the following labels define the sizes of
// the transmit and receive buffers respectively.
//
//*****************************************************************************
#ifdef UART_BUFFERED1
#ifndef UART_RX_BUFFER_SIZE1
#define UART_RX_BUFFER_SIZE1     128
#endif
#ifndef UART_TX_BUFFER_SIZE1
#define UART_TX_BUFFER_SIZE1     1024
#endif
#endif

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void UARTStdioConfig1(uint32_t ui32Port, uint32_t ui32Baud,
                            uint32_t ui32SrcClock);
extern int UARTgets1(char *pcBuf, uint32_t ui32Len);
extern unsigned char UARTgetc1(void);
extern void UARTprintf1(const char *pcString, ...);
extern void UARTvprintf1(const char *pcString, va_list vaArgP);
extern int UARTwrite1(const char *pcBuf, uint32_t ui32Len);
#ifdef UART_BUFFERED1
extern int UARTPeek1(unsigned char ucChar);
extern void UARTFlushTx1(bool bDiscard);
extern void UARTFlushRx1(void);
extern int UARTRxBytesAvail1(void);
extern int UARTTxBytesFree1(void);
extern void UARTEchoSet1(bool bEnable);
#endif

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __UARTSTDIO_H__
