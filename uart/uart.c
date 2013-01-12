/*****************************************************************************
* File          : uart.c
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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "uart.h"

// real size of RX/TX buffers
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)

#if defined(__AVR_ATmega48__)  || defined(__AVR_ATmega88__)  || defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega48P__) || defined(__AVR_ATmega88P__) || defined(__AVR_ATmega168P__) || \
    defined(__AVR_ATmega328P__)
 // ATmega with one USART
 #define ATMEGA_USART0
 #define UART0_RECEIVE_INTERRUPT   USART_RX_vect
 #define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
 #define UART0_STATUS   UCSR0A
 #define UART0_CONTROL  UCSR0B
 #define UART0_DATA     UDR0
 #define UART0_UDRIE    UDRIE0
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega640__)
// ATmega with two USART
  #define ATMEGA_USART0
  #define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
  #define UART0_TRANSMIT_INTERRUPT  USART1_RX_vect
  #define UART0_STATUS   UCSR0A
  #define UART0_CONTROL  UCSR0B
  #define UART0_DATA     UDR0
  #define UART0_UDRIE    UDRIE0
  #define ATMEGA_USART1
  #define UART1_TRANSMIT_INTE, DOR=3RRUPT  USART1_UDRE_vect
  #define UART1_RECEIVE_INTERRUPT   USART0_UDRE_vect
  #define UART1_STATUS   UCSR1A
  #define UART1_CONTROL  UCSR1B
  #define UART1_DATA     UDR1
  #define UART1_UDRIE    UDRIE1  
#else
 #error "no UART definition for MCU available"
#endif


// global variables, only visible here (in this file)
static volatile uint8_t UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile uint8_t UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile uint8_t UART_TxHead;
static volatile uint8_t UART_TxTail;
static volatile uint8_t UART_RxHead;
static volatile uint8_t UART_RxTail;
static volatile uint8_t UART_LastRxError;

#if defined( ATMEGA_USART1 )
	static volatile uint8_t UART1_TxBuf[UART_TX_BUFFER_SIZE];
	static volatile uint8_t UART1_RxBuf[UART_RX_BUFFER_SIZE];
	static volatile uint8_t UART1_TxHead;
	static volatile uint8_t UART1_TxTail;
	static volatile uint8_t UART1_RxHead;
	static volatile uint8_t UART1_RxTail;
	static volatile uint8_t UART1_LastRxError;
#endif


/*************************************************************************
	UART Receive Complete interrupt
	called when the UART has received a character
**************************************************************************/
ISR(UART0_RECEIVE_INTERRUPT){

	uint8_t data;
	uint8_t usr;
	uint8_t lastRxError;

	// read UART status register and UART data register
	usr  = UART0_STATUS;
	data = UART0_DATA;

#if defined( ATMEGA_USART0 )
	lastRxError = (usr & ((1<<FE0)|(1<<DOR0)));
#endif

	// store received data in buffer even if buffer is overflow
	UART_RxBuf[UART_RxHead] = data;
	UART_RxHead = (UART_RxHead + 1) % UART_RX_BUFFER_MASK;

    if( UART_RxHead == UART_RxTail ) {
    	lastRxError = UART_BUFFER_OVERFLOW >> 8;
    	UART_RxTail = (UART_RxTail + 1) % UART_RX_BUFFER_MASK;
    }

    UART_LastRxError = lastRxError;
}


/*************************************************************************
	UART Data Register Empty interrupt
	called when the UART is ready to transmit the next byte
**************************************************************************/
ISR(UART0_TRANSMIT_INTERRUPT) {

    if(UART_TxHead != UART_TxTail) {
    	UART0_DATA = UART_TxBuf[UART_TxTail];
    	UART_TxTail = (UART_TxTail + 1) % UART_TX_BUFFER_MASK;
    } else {
        // tx buffer empty, disable UDRE interrupt
        UART0_CONTROL &= ~(1<<UART0_UDRIE);
    }
}


/*************************************************************************
	init uart (set baudrate, enable interrupts, data 8n1)
**************************************************************************/
void uart_init(uint32_t baudrate) {

    UART_TxHead = 0;
    UART_TxTail = 0;
    UART_RxHead = 0;
    UART_RxTail = 0;

    uint16_t ubrr = ((F_CPU/(16 * baudrate)) - 0.5);

    UBRR0H  = (uint8_t)(ubrr>>8);
    UBRR0L  = (uint8_t)(ubrr);

    // Enable USART receiver and transmitter and receive complete interrupt
    UART0_CONTROL = (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0);

    // Set frame format: asynchronous, 8data, no parity, 1stop bit
    UCSR0C = (3<<UCSZ00);
}


/*************************************************************************
	return byte from circular buffer
	low byte:  received byte from circular buffer
    high byte: last receive error
**************************************************************************/
unsigned int uart_getc(void) {

    unsigned char data;

    if ( UART_RxHead == UART_RxTail ) {
        return UART_NO_DATA; // no data available
    }
    
    data = UART_RxBuf[UART_RxTail];
    UART_RxTail = (UART_RxTail + 1) % UART_RX_BUFFER_MASK;

    return (UART_LastRxError << 8) + data;
}


/*************************************************************************
	write byte to circular buffer for transmitting via UART
**************************************************************************/
void uart_putc(char data) {

	while( ((UART_TxHead + 1) % UART_TX_BUFFER_MASK ) == UART_TxTail ){
		;// wait for free space in buffer
	}

    UART_TxBuf[UART_TxHead] = data;
    UART_TxHead = (UART_TxHead + 1) % UART_TX_BUFFER_MASK;

    // enable UDRE interrupt
    UART0_CONTROL |= (1<<UART0_UDRIE);
}


/*************************************************************************
	write 2 bytes (A and B) to circular buffer representing b in hex (0xAB)
**************************************************************************/
void uart_putc_hex(uint8_t b) {

    /* upper nibble */
    if((b >> 4) < 0x0a) //160
        uart_putc((b >> 4) + '0');
    else
        uart_putc((b >> 4) - 0x0a + 'a');

    /* lower nibble */
    if((b & 0x0f) < 0x0a)
        uart_putc((b & 0x0f) + '0');
    else
        uart_putc((b & 0x0f) - 0x0a + 'a');
}


/*************************************************************************
	write 4 bytes (A, B, C, D) to circular buffer
	representing word in hex (0xABCD)
**************************************************************************/
void uart_putw_hex(uint16_t w) {

    uart_putc_hex((uint8_t) (w >> 8));
    uart_putc_hex((uint8_t) (w & 0xff));
}


/*************************************************************************
	write 6 bytes (A, B, C, D, E, F) to circular buffer
	representing double word in hex (0xABCDEF)
**************************************************************************/
void uart_putdw_hex(uint32_t dw) {

    uart_putw_hex((uint16_t) (dw >> 16));
    uart_putw_hex((uint16_t) (dw & 0xffff));
}


/*************************************************************************
	writes decimal number as 'characters'
**************************************************************************/
void uart_putw_dec(uint16_t w) {

    uint16_t num = 10000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = w / num;
        if(b > 0 || started || num == 1)
        {
            uart_putc('0' + b);
            started = 1;
        }
        w -= b * num;

        num /= 10;
    }
}


/*************************************************************************
	writes decimal number as 'characters'
**************************************************************************/
void uart_putdw_dec(uint32_t dw) {

    uint32_t num = 1000000000;
    uint8_t started = 0;

    while(num > 0)
    {
        uint8_t b = dw / num;
        if(b > 0 || started || num == 1)
        {
            uart_putc('0' + b);
            started = 1;
        }
        dw -= b * num;

        num /= 10;
    }
}


/*************************************************************************
	transmit string to UART
**************************************************************************/
void uart_puts(const char *s ){

    while (*s){
    	uart_putc(*s++);
    }
}


/*************************************************************************
	transmit string from program memory to UART
**************************************************************************/
void uart_puts_p(const char *progmem_s ){

    register char c;
    
    while( (c = pgm_read_byte(progmem_s++)) ){
      uart_putc(c);
    }
}


/*************************************************************************
	determine the number of bytes waiting in the receive buffer
**************************************************************************/
unsigned int uart_available(void){

	return (UART_RX_BUFFER_MASK + UART_RxHead - UART_RxTail) % UART_RX_BUFFER_MASK;
}


/*************************************************************************
	flush bytes waiting the receive buffer, actually ignores them
**************************************************************************/
void uart_flush(void){

    UART_RxHead = UART_RxTail;
}


/*
 *	only for ATmegas with two USART
 *	TODO: implement and test all methods from above
 */
#if defined( ATMEGA_USART1 )
/*************************************************************************
	UART Receive Complete interrupt
	called when the UART has received a character
**************************************************************************/
ISR(UART1_RECEIVE_INTERRUPT){

	uint8_t data;
	uint8_t usr;
	uint8_t lastRxError;

	// read UART status register and UART data register
	usr  = UART1_STATUS;
	data = UART1_DATA;

#if defined( ATMEGA_USART1 )
	lastRxError = (usr & ((1<<FE1)|(1<<DOR1)));
#endif

	// store received data in buffer even if buffer is overflow
	UART_RxBuf[UART_RxHead] = data;
	UART_RxHead = (UART_RxHead + 1) % UART_RX_BUFFER_MASK;

    if( UART_RxHead == UART_RxTail ) {
    	lastRxError = UART_BUFFER_OVERFLOW >> 8;
    	UART_RxTail = (UART_RxTail + 1) % UART_RX_BUFFER_MASK;
    }

    UART_LastRxError = lastRxError;
}


/*************************************************************************
	UART Data Register Empty interrupt
	called when the UART is ready to transmit the next byte
**************************************************************************/
ISR(UART1_TRANSMIT_INTERRUPT) {

    if(UART_TxHead != UART_TxTail) {
    	UART0_DATA = UART_TxBuf[UART_TxTail];
    	UART_TxTail = (UART_TxTail + 1) % UART_TX_BUFFER_MASK;
    } else {
        // tx buffer empty, disable UDRE interrupt
        UART1_CONTROL &= ~(1<<UART1_UDRIE);
    }
}


/*************************************************************************
	init uart (set baudrate, enable interrupts, data 8n1)
**************************************************************************/
void uart1_init(uint32_t baudrate) {

    UART_TxHead = 0;
    UART_TxTail = 0;
    UART_RxHead = 0;
    UART_RxTail = 0;

    uint16_t ubrr = ((F_CPU/(16 * baudrate)) - 0.5);

    UBRR1H  = (uint8_t)(ubrr>>8);
    UBRR1L  = (uint8_t)(ubrr);

    // Enable USART receiver and transmitter and receive complete interrupt
    UART1_CONTROL = (1<<RXCIE1) | (1<<RXEN1) | (1<<TXEN1);

    // Set frame format: asynchronous, 8data, no parity, 1stop bit
    UCSR1C = (3<<UCSZ01);
}


/*************************************************************************
	return byte from circular buffer
	low byte:  received byte from circular buffer
    high byte: last receive error
**************************************************************************/
unsigned int uart1_getc(void) {

    unsigned char data;

    if ( UART_RxHead == UART_RxTail ) {
        return UART_NO_DATA; // no data available
    }

    data = UART_RxBuf[UART_RxTail];
    UART_RxTail = (UART_RxTail + 1) % UART_RX_BUFFER_MASK;

    return (UART_LastRxError << 8) + data;
}


/*************************************************************************
	write byte to circular buffer for transmitting via UART
**************************************************************************/
void uart1_putc(unsigned char data) {

	while( ((UART_TxHead + 1) % UART_TX_BUFFER_MASK ) == UART_TxTail ){
		;// wait for free space in buffer
	}

    UART_TxBuf[UART_TxHead] = data;
    UART_TxHead = (UART_TxHead + 1) % UART_TX_BUFFER_MASK;

    // enable UDRE interrupt
    UART1_CONTROL |= (1<<UART1_UDRIE);
}


/*************************************************************************
	transmit string to UART
**************************************************************************/
void uart1_puts(const char *s ){

    while (*s){
    	uart_putc(*s++);
    }
}


/*************************************************************************
	transmit string from program memory to UART
**************************************************************************/
void uart1_puts_p(const char *progmem_s ){

    register char c;

    while( (c = pgm_read_byte(progmem_s++)) ){
      uart_putc(c);
    }
}


/*************************************************************************
	determine the number of bytes waiting in the receive buffer
**************************************************************************/
unsigned int uart1_available(void){

	return (UART_RX_BUFFER_MASK + UART_RxHead - UART_RxTail) % UART_RX_BUFFER_MASK;
}


/*************************************************************************
	flush bytes waiting the receive buffer, actually ignores them
**************************************************************************/
void uart1_flush(void){

    UART_RxHead = UART_RxTail;
}
#endif
