/*
 * dwire_pins.h
 *
 *  Created on: 30 May 2016
 *      Author: Stefan van der Linden
 */

#ifndef INCLUDE_DWIRE_PINS_H_
#define INCLUDE_DWIRE_PINS_H_

//****************************************************************************
// MSP432 devices
//****************************************************************************
#if defined (__MSP432P401R__)
#include <dwire_msp432p401r.h>
//****************************************************************************
// Failed to match a default include file
//****************************************************************************
#else
#error "Failed to match a default include file"
#endif

#endif /* INCLUDE_DWIRE_PINS_H_ */
