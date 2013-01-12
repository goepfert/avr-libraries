/*
 * 	spi.h
 *
 *  Created on: Nov 30, 2012
 *      Author: thomas.goepfert@solderlab.de
 *     Verison: 0.1
 *
 *     Usage: 	initSPI(SPI_MODE_0, SPI_MSB, SPI_NO_INTERRUPT, SPI_MSTR_CLK128);
 *     		  	sendSPI(0xF2);
 */

#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>

// ATmega328
#define SPI_SS_PIN PORTB2
#define SPI_SCK_PIN PORTB5
#define SPI_MOSI_PIN PORTB3
#define SPI_MISO_PIN PORTB4

// SPI clock modes
#define SPI_MODE_0 0x00 // Sample (Rising) Setup (Falling) CPOL=0, CPHA=0
#define SPI_MODE_1 0x01 // Setup (Rising) Sample (Falling) CPOL=0, CPHA=1
#define SPI_MODE_2 0x02 // Sample (Falling) Setup (Rising) CPOL=1, CPHA=0
#define SPI_MODE_3 0x03 // Setup (Falling) Sample (Rising) CPOL=1, CPHA=1

// data direction
#define SPI_LSB 1 // send least significant bit (bit 0) first
#define SPI_MSB 0 // send most significant bit (bit 7) first

// whether to raise interrupt when data received (SPIF bit received)
#define SPI_NO_INTERRUPT 0
#define SPI_INTERRUPT 1

// clock divider
#define SPI_MSTR_CLK4 0x00   // chip clock/4
#define SPI_MSTR_CLK16 0x01  // chip clock/16
#define SPI_MSTR_CLK64 0x02  // chip clock/64
#define SPI_MSTR_CLK128 0x03 // chip clock/128
#define SPI_MSTR_CLK2 0x04   // chip clock/2
#define SPI_MSTR_CLK8 0x05   /* chip clock/8 */
#define SPI_MSTR_CLK32 0x06  /* chip clock/32 */

void initSPI(uint8_t mode,   // timing mode SPI_MODE[0-4]
             int dord,       // data direction SPI_LSB|SPI_MSB
             int interrupt,  // whether to raise interrupt on receive
             uint8_t clock); // clock divider

void disableSPI(void);
uint8_t sendSPI(uint8_t out);

#endif /* SPI_H_ */
