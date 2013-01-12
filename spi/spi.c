/*
 * 	spi.c
 *
 *  Created on: Nov 30, 2012
 *      Author: thomas.goepfert@solderlab.de
 *     Version: 0.1
 */

#include "spi.h"

void initSPI(uint8_t mode, int dord, int interrupt, uint8_t clock) {

  // specify pin directions for SPI pins on port B
  DDRB |= (1 << SPI_MOSI_PIN);  // output
  DDRB &= ~(1<<SPI_MISO_PIN);   // input
  DDRB |= (1 << SPI_SCK_PIN);   // output
  DDRB |= (1 << SPI_SS_PIN);    // output

  SPCR = ((interrupt ? 1 : 0) << SPIE) // interrupt enabled
    | (1 << SPE)                       // enable SPI
    | (dord << DORD)                   // LSB or MSB
    | (1 << MSTR)                      // Master
    | (((mode & 0x02) == 2) << CPOL)   // clock timing mode CPOL
    | (((mode & 0x01)) << CPHA)        // clock timing mode CPHA
    | (((clock & 0x02) == 2) << SPR1)  // cpu clock divisor SPR1
    | ((clock & 0x01) << SPR0);        // cpu clock divisor SPR0

  SPSR = (((clock & 0x04) == 4) << SPI2X);    // clock divisor SPI2X
}

void disableSPI() {

  SPCR = 0;
}

uint8_t sendSPI(uint8_t out) {

  SPDR = out;
  while (!(SPSR & (1<<SPIF)));
  return SPDR;
}
