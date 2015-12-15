/**
 * @file extra.h
 * Library including functions useful in embedded progamming.
 */

#ifndef extra_h
#define extra_h

#include "MKL46Z4.h"   /* Device header */	

/**
 * Delay the operation of the MCU for value*10 000 clock cycles.
 * @param[in] value number of clock cyckles*10 000 to delay the operation of MCU. 
 */
void delay_mc(uint32_t value);

#endif
