#ifndef pit_h
#define pit_h

#include "MKL46Z4.h"
#include "uart.h"
#include "i2c_mpu6050.h"
#include "mpu6050.h"

#define PIT_IRQ_NBR (IRQn_Type) 22 // Define the number of interrupt vector for Periodic Interrupt Timer

void pit_init(void);

static uint8_t distance_cm = 0;

#endif
