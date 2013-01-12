/*****************************************************************************
* File          : uart.h
* Created       : 12.01.2013
*
* Title         : ---
* Author        : Thomas Goepfert
* Contact       : info@SolderLab.de
*
* Version       : 1.0
* Last Changed  : 12.01.2013 by goepfert
* Remarks       : Based on Peter Fleury's uart lib
*
* Description   : uart with circular buffers
*
*****************************************************************************/

#ifndef UART_H
#define UART_H

#include <avr/pgmspace.h>

#ifndef UART_RX_BUFFER_SIZE
	#define UART_RX_BUFFER_SIZE 16
#endif

#ifndef UART_TX_BUFFER_SIZE
	#define UART_TX_BUFFER_SIZE 16
#endif

// error return code: high byte of uart_getc()
#define UART_FRAME_ERROR      0x0800 // Framing Error by UART - FE=4
#define UART_OVERRUN_ERROR    0x0400 // Overrun condition by UART - DOR=3
									 // A character already present in the UART UDR register was 
 	 	 	 	 	 	 	 	 	 // not read by the interrupt handler before the next character arrived,
									 // one or more received characters have been dropped.
#define UART_BUFFER_OVERFLOW  0x0200 // receive circular buffer overflow
 	 	 	 	 	 	 	 	 	 // We are not reading the receive buffer fast enough, 
 	 	 	 	 	 	 	 	 	 // one or more received character have been dropped.
#define UART_NO_DATA          0x0100 // no receive data available

// function prototypes
extern void uart_init(uint32_t baudrate);
extern unsigned int uart_getc(void); 	   // low byte: received character, high byte: last receive error
extern void uart_putc(char data); 		   // put char into transmit circular buffer
extern void uart_putc_hex(uint8_t b);	   // represent byte in hex format (0xAB) and put into circular buffer
extern void uart_putw_hex(uint16_t w);     // uint16_t to hex (0xABCD)
extern void uart_putdw_hex(uint32_t dw);   // uint32_t to hex (0xABCDEF)
extern void uart_putw_dec(uint16_t w);     // decimal number as 'characters'
extern void uart_putdw_dec(uint32_t dw);   // decimal number as 'characters'
extern void uart_puts(const char *s );     // blocks if it can not write the whole string into the circular buffer
extern void uart_puts_p(const char *s );   // put string from program memory to circular buffer
extern unsigned int uart_available(void);  // bytes waiting in the receive buffer
extern void uart_flush(void); 			   // flush bytes waiting in receive buffer

#define uart_puts_P(__s) 	uart_puts_p(PSTR(__s)) // Macro to automatically put a string constant into program memory

// initialize USART1 (only available on selected ATmegas), see uart_init
extern void uart1_init(unsigned int baudrate);
extern unsigned int uart1_getc(void);
extern void uart1_putc(unsigned char data);
extern void uart1_puts(const char *s );
extern void uart1_puts_p(const char *s );
extern unsigned int uart1_available(void);
extern void uart1_flush(void);

#define uart1_puts_P(__s)	uart1_puts_p(PSTR(__s))

#endif // UART_H 
