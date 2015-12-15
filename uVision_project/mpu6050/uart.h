/**
 * @addtogroup  DRIVERS Sensor Driver Layer
 * @brief       Hardware drivers to communicate with sensors via I2C.
 * @{
 * 		@file  uart.h
 *		Communication via UART for FRDM-MKL46Z board.
 * 		For this code to work properly the "CLOCK_SETUP" must be set to 1:\n
 *   		1 ... Multipurpose Clock Generator (MCG) in PLL Engaged External (PEE) mode
 *         	      Reference clock source for MCG module is an external crystal 8MHz
 *         	      Core clock = 48MHz, BusClock = 24MHz. 
 */

#ifndef UART_H
#define UART_H

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
 * @return data received from UART communication
 */
uint8_t uart_receive_poll(void);

/**
 * Send the data by interrupt-driven communication.
 * @param[in] data
 */
void uart_transmit(const char *data, ...);

/**
 * Read the data from the UART interface.
 * @param[out] data buffer in which to store the received data.
 */
void uart_receive(char *data);

/**
 * IRQ Handler for the interrupt-driven communication.
 */
void UART0_IRQHandler(void);

#endif

/**@}*/
