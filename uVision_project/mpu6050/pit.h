#ifndef pit_h
#define pit_h

#include "MKL46Z4.h"
#include "uart.h"
#include "i2c_mpu6050.h"
#include "mpu6050.h"
#include "stdbool.h"
#include "math.h" // rint()

#define PIT_IRQ_NBR (IRQn_Type) 22 // Define the number of interrupt vector for Periodic Interrupt Timer

/**
  * Initialization function for the Periodic Interrupt Timer.
  */
void pit_init(void);

/**
  * Function for delaying program executino for the given amount of time.
  * @param[in] ms_to_wait number of milliseonds to wait
  */
void wait_ms(uint32_t ms_to_wait);

#endif
