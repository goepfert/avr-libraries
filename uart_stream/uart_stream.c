/*****************************************************************************
 * File          : uart_stream.c
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
 *                 use printf for writing to uart *
 *****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "uart_stream.h"

// real size of RX/TX buffers
#define UART_STREAM_RX_BUFFER_MASK (UART_STREAM_RX_BUFFER_SIZE - 1)
#define UART_STREAM_TX_BUFFER_MASK (UART_STREAM_TX_BUFFER_SIZE - 1)

#if defined(__AVR_ATmega1284P__)
/* ATmega with two USART */
#define ATMEGA_USART0
#define ATMEGA_USART1
#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
#define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
#define UART0_STATUS   UCSR0A
#define UART0_CONTROL  UCSR0B
#define UART0_DATA     UDR0
#define UART0_UDRIE    UDRIE0
#define UART1_STATUS   UCSR1A
#define UART1_CONTROL  UCSR1B
#define UART1_DATA     UDR1
#define UART1_UDRIE    UDRIE1
#else
#error "no UART definition for MCU available"
#endif

// global variables, only visible here (in this file)
static volatile uint8_t UART_STREAM_TxBuf[UART_STREAM_TX_BUFFER_SIZE];
static volatile uint8_t UART_STREAM_RxBuf[UART_STREAM_RX_BUFFER_SIZE];
static volatile uint8_t UART_STREAM_TxHead;
static volatile uint8_t UART_STREAM_TxTail;
static volatile uint8_t UART_STREAM_RxHead;
static volatile uint8_t UART_STREAM_RxTail;
static volatile uint8_t UART_STREAM_LastRxError;


/*************************************************************************
* UART Receive Complete interrupt
* called when the UART has received a character
**************************************************************************/
ISR(UART0_RECEIVE_INTERRUPT) {

	uint8_t data;
	uint8_t usr;
	uint8_t lastRxError;

	// read UART status register and UART data register
	usr = UART0_STATUS;
	data = UART0_DATA;

#if defined( ATMEGA_USART0 )
	lastRxError = (usr & ((1 << FE0) | (1 << DOR0)));
#endif

	// store received data in buffer even if buffer is overflow
	UART_STREAM_RxBuf[UART_STREAM_RxHead] = data;
	UART_STREAM_RxHead = (UART_STREAM_RxHead + 1) % UART_STREAM_RX_BUFFER_MASK;

	if (UART_STREAM_RxHead == UART_STREAM_RxTail) {
		lastRxError = UART_STREAM_BUFFER_OVERFLOW >> 8;
		UART_STREAM_RxTail = (UART_STREAM_RxTail + 1) % UART_STREAM_RX_BUFFER_MASK;
	}

	UART_STREAM_LastRxError = lastRxError;
}

/*************************************************************************
* UART Data Register Empty interrupt
* called when the UART is ready to transmit the next byte
**************************************************************************/
ISR(UART0_TRANSMIT_INTERRUPT) {

	if (UART_STREAM_TxHead != UART_STREAM_TxTail) {
		UART0_DATA = UART_STREAM_TxBuf[UART_STREAM_TxTail];
		UART_STREAM_TxTail = (UART_STREAM_TxTail + 1) % UART_STREAM_TX_BUFFER_MASK;
	} else {
		// tx buffer empty, disable UDRE interrupt
		UART0_CONTROL &= ~(1 << UART0_UDRIE);
	}
}

/*************************************************************************
 * init uart (set baudrate, enable interrupts, data 8n1)
 *************************************************************************/
void uart_init(uint32_t baudrate) {

	UART_STREAM_TxHead = 0;
	UART_STREAM_TxTail = 0;
	UART_STREAM_RxHead = 0;
	UART_STREAM_RxTail = 0;

	uint16_t ubrr = ((F_CPU / (16 * baudrate)) - 0.5);

	UBRR0H = (uint8_t) (ubrr >> 8);
	UBRR0L = (uint8_t) (ubrr);

	// Enable USART receiver and transmitter and receive complete interrupt
	UART0_CONTROL = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);

	// Set frame format: asynchronous, 8data, no parity, 1stop bit
	UCSR0C = (3 << UCSZ00);
}

/*************************************************************************
* return byte from circular buffer
* low byte:  received byte from circular buffer
* high byte: last receive error
**************************************************************************/
unsigned int uart_getc(void) {

	unsigned char data;

	if (UART_STREAM_RxHead == UART_STREAM_RxTail) {
		return UART_STREAM_NO_DATA; // no data available
	}

	data = UART_STREAM_RxBuf[UART_STREAM_RxTail];
	UART_STREAM_RxTail = (UART_STREAM_RxTail + 1) % UART_STREAM_RX_BUFFER_MASK;

	return (UART_STREAM_LastRxError << 8) + data;
}

/*************************************************************************
 * write byte to circular buffer for transmitting via UART
 *************************************************************************/
void uart_putc(char data) {

	while (((UART_STREAM_TxHead + 1) % UART_STREAM_TX_BUFFER_MASK)== UART_STREAM_TxTail ) {
		; // wait for free space in buffer
	}

	UART_STREAM_TxBuf[UART_STREAM_TxHead] = data;
	UART_STREAM_TxHead = (UART_STREAM_TxHead + 1) % UART_STREAM_TX_BUFFER_MASK;

	// enable UDRE interrupt
	UART0_CONTROL |= (1 << UART0_UDRIE);
}


int uart_putstream(char data, FILE *stream) {

	uart_putc(data);
	return 0;
}


/*************************************************************************
 * transmit string from program memory to UART
 *************************************************************************/
void uart_puts_p(const char *progmem_s) {

	register char c;

	while ((c = pgm_read_byte(progmem_s++))) {
		uart_putc(c);
	}
}

/*************************************************************************
 * determine the number of bytes waiting in the receive buffer
 *************************************************************************/
unsigned int uart_available(void) {

	return (UART_STREAM_RX_BUFFER_MASK + UART_STREAM_RxHead - UART_STREAM_RxTail)
			% UART_STREAM_RX_BUFFER_MASK;
}

/*************************************************************************
 * flush bytes waiting the receive buffer, actually ignores them
 *************************************************************************/
void uart_flush(void) {

	UART_STREAM_RxHead = UART_STREAM_RxTail;
}

