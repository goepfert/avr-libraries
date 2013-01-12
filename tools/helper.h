/*****************************************************************************
* File          : helper.h
* Created       : 12.01.2013
*
* Title         : ---
* Author        : Thomas Goepfert
* Contact       : info@SolderLab.de
*
* Version       : 1.0
* Last Changed  : 12.01.2013 by goepfert
* Remarks       : ---
*
* Description   : simple bit manipulation
*
*****************************************************************************/

#ifndef HELPER_H_
#define HELPER_H_

#include <avr/io.h>
#include <avr/interrupt.h>

//-- Bit Manipulations ----------------------------------------------//

#define out(x)			_out(x)
#define _out(bit,port)	DDR##port |= (1 << bit)

#define in(x)			_in(x)
#define _in(bit,port)	DDR##port &= ~(1 << bit)

#define on(x)			_on(x)
#define _on(bit,port)	PORT##port |= (1 << bit)

#define off(x)			_off(x)
#define _off(bit,port)	PORT##port &= ~(1 << bit)

#define toggle(x)			_toggle(x)
#define _toggle(bit,port)	PORT##port ^= (1 << bit)

#define pullup(x)		_on(x)
#define high(x)			_on(x)
#define low(x)			_off(x)

#define get(x)			_get(x)
#define _get(bit,port)	(PIN##port & (1 << bit))

#endif /* HELPER_H_ */
