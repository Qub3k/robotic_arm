/**
* @file  uart.h
*
* Communication via UART
*/

/**
 * @defgroup UART UART communication setup and configuration
 * @{
 */

#ifndef UART_H
#define UART_H

/**
 * For this code to work properly the "CLOCK_SETUP" must be set to 1:
 *   1 ... Multipurpose Clock Generator (MCG) in PLL Engaged External (PEE) mode
 *         Reference clock source for MCG module is an external crystal 8MHz
 *         Core clock = 48MHz, BusClock = 24MHz. 
 */

#include "MKL46Z4.h"
#include "queue.h"
#include <cstring> // strlen();
#include <cstdarg> // va_list, va_start, va_end
#include <cstdio> // vsprintf()

/**
* Maximum buffer size used for transmission.
*/
#define MAX_BUFF_SIZE 64 

/**
* Initializatino function.
*/
void uart_init(void);

/**
* Send the data by polling the Transmitter Empty flag.
* @param[in] data 
*/
void uart_transmit_poll(uint8_t data);

/**
* Receive the data by polling the Receiver Full flag.
*/
uint8_t uart_receive_poll(void);

/**
* Send the data by interrupt-driven communication.
* @param[in] data
*/
void uart_transmit(const char *data, ...);

/**
* Read the data from the queue.
* @param[in] data
*/
void uart_receive(char *data);

/**
* IRQ Handler for the interrupt-driven communication.
*/
void UART0_IRQHandler(void);

#endif
<<<<<<< HEAD

/**@}*/
/**@}*/
=======
>>>>>>> c90e81a5e6f1e21f223effaa2104e25dd52cece4
