/*****************************************************************************
* File          : uart_stream.h
* Created       : 23.03.2013
*
* Title         : ---
* Author        : Thomas Goepfert
* Contact       : info@SolderLab.de
*
* Version       : 1.0
* Last Changed  : 23.03.2013 by goepfert
* Remarks       : Based on Peter Fleury's uart lib
*
* Description   : uart with circular buffers using stdout
* 				  you need to redirect stdout = &mystdout
*                 use printf for writing to uart
*****************************************************************************/

#ifndef UART_STREAM_H
#define UART_STREAM_H

#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdint.h>

#ifndef UART_STREAM_RX_BUFFER_SIZE
	#define UART_STREAM_RX_BUFFER_SIZE 64
#endif

#ifndef UART_STREAM_TX_BUFFER_SIZE
	#define UART_STREAM_TX_BUFFER_SIZE 16
#endif

// error return code: high byte of uart_stream_getc()
#define UART_STREAM_FRAME_ERROR      0x0800 	// Framing Error by UART - FE=4
#define UART_STREAM_OVERRUN_ERROR    0x0400 	// Overrun condition by UART - DOR=3
												// A character already present in the UART UDR register was
 	 	 	 	 	 	 	 	 	 	 	 	// not read by the interrupt handler before the next character arrived,
									 	 	 	// one or more received characters have been dropped.
#define UART_STREAM_BUFFER_OVERFLOW  0x0200 	// receive circular buffer overflow
 	 	 	 	 	 	 	 	 	 	 	 	// We are not reading the receive buffer fast enough,
 	 	 	 	 	 	 	 	 	 	 	 	// one or more received character have been dropped.
#define UART_STREAM_NO_DATA          0x0100 	// no receive data available


// function prototypes
extern void uart_init(uint32_t baudrate);
extern unsigned int uart_getc(void); 	   			// low byte: received character, high byte: last receive error
extern void uart_putc(char data);
extern int uart_putstream(char data, FILE *stream);
extern void uart_puts_p(const char *s );   			// put string from program memory to circular buffer
extern unsigned int uart_available(void);  			// bytes waiting in the receive buffer
extern void uart_flush(void); 			   			// flush bytes waiting in receive buffer

static FILE mystdout = FDEV_SETUP_STREAM( uart_putstream, NULL, _FDEV_SETUP_WRITE );

#define uart_puts_P(__s) 	uart_puts_p(PSTR(__s)) // Macro to automatically put a string constant into program memory

#endif // UART_STREAM_H
