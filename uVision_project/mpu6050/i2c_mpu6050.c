#include "i2c_mpu6050.h"

void i2c_init(void) {
  /* Enable clock to the pins used as SCL(PTE1) and SDA(PTE0) */
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  
  /* Enable clock to I2C => I2C0 is driven by the Bus Clock/2 = 12 MHz*/
  SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
  
  /* Set MUX configuration of PTE0 and PTE1 to I2C */
  PORTE->PCR[0] |= PORT_PCR_MUX(6);
  PORTE->PCR[1] |= PORT_PCR_MUX(6);
  
  /* Minimum "SCL start hold time" for MPU-6050 is 600 ns so we may set it to 9.833 us to be sure 
   * that everything is OK.
   * SCL start hold time = delay from fallinge edge of SDA(while SCL is high = start condition) 
   *                       to falling edge of SCL (end of START signal)
   * Minimum "SCL stop hold time" is 0.6 us = 600 ns so we may make it approx. 10.083 us.
   * Minimum "SDA hold time" is 0 us => we set it to 2.7499 us 
   * The resulting Baud Rate is 50 kBd/s = 50 kbit/s */
  I2C0->F &= ~I2C_F_MULT_MASK; // mul = 1
  I2C0->F |= I2C_F_ICR(0x1F); // SCL divider = 240
  
  /* Enable I2C module */
  I2C0->C1 |= I2C_C1_IICEN_MASK;
}

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data) {
  // 1. Send the Start bit
	// 2. Send the 7-bits long "slave_addr" + Write bit
	// 3. Wait for 1 bit of acknowledgment
	// 4. Send the 8-bits long "reg_addr"
	// 5. Wait for 1 bit of acknowledgment
	// 6. Send 8-bits long data from "data" pointer
  unsigned int i = 0;
  
  /* Define the direction of communication as transmission */
  I2C0->C1 |= I2C_C1_TX_MASK; // choose the "transmit" mode
  
  /* Generate a START signal */
  I2C0->C1 |= I2C_C1_MST_MASK;
  
  /* Write the slave address + WRITE bit to the "DATA" register and send it*/
  I2C0->D = (slave_addr << 1) & ~1; 
  
  /* Exit if NACK signal detected */
  if(I2C0->S & I2C_S_RXAK_MASK) {
    /* Send the STOP signal and exit with the error*/
    I2C0->C1 &= ~I2C_C1_MST_MASK;
    return -1;
  }
  
  /* Send the register address */
  I2C0->D = reg_addr;
  
  /* Exit if NACK signal detected */
  if(I2C0->S & I2C_S_RXAK_MASK) {
    /* Send the STOP signal and exit with the error*/
    I2C0->C1 &= ~I2C_C1_MST_MASK;
    return -1;
  }
  
  /* Send the appropriate number of bytes of data */
  for(i = 0; i < length; i++) {
    I2C0->D = data[i]; // send the data
    
    /* Wait for 200 us - probably not neccessary*/
    delay_mc(1);
    
    /* Exit if NACK signal detected */
    if(I2C0->S & I2C_S_RXAK_MASK) {
      /* Send the STOP signal and exit with the error*/
      I2C0->C1 &= ~I2C_C1_MST_MASK;
      return -1;
    }
  }
  
  /* Generate a STOP signal */
  I2C0->C1 &= ~I2C_C1_MST_MASK;
	  
  return 0;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
  // 1. Send the Start bit
	// 2. Send the 7-bits long "slave_addr" + WRITE bit
	// 3. Wait for 1 bit of acknowledgment
	// 4. Send the 8-bits long "reg_addr"
  // 5. 3. Wait for 1 bit of acknowledgment
  // 6. Send another Start bit 
  // 7. Send the 7-bits long "slave_addr" + READ bit
  // 8. Wait for 1 bit of acknowledgment
  // 9. Read 8-bits long data and store in proper position of data
	// 10. Send the NACK signal
  // 11. Send the STOP signal
  unsigned int i = 0;
  
  /* Define the direction of communication as transmission */
  I2C0->C1 |= I2C_C1_TX_MASK; // choose the "transmit" mode
  
  /* Generate a START signal */
  I2C0->C1 |= I2C_C1_MST_MASK;
  
  /* Write the slave address to the "DATA" register + WRITE bit */
  I2C0->D = (slave_addr << 1) & ~1; // put down the slave address + WRITE mode
  
  /* Exit if NACK signal detected */
  if(I2C0->S & 1)
    return -1;
  
  /* Send the register address */
  I2C0->D = reg_addr;
  
  /* Exit if NACK signal detected */
  if(I2C0->S & 1)
    return -1;
  
  /* Generate a Repeated START signal */
  I2C0->C1 |= I2C_C1_RSTA_MASK;
  
  /* Write the slave address to the "DATA" register + READ bit */
  I2C0->D = (slave_addr << 1) | 1; // put down the slave address + READ mode\
  
  /* Exit if NACK signal detected */
  if(I2C0->S & 1)
    return -1;
  
  /* Change the direction of communication */
  I2C0->C1 &= ~I2C_C1_TX_MASK; // choose the "receive" mode
  
  /* Read 8-bits of data and store it in the data variable */
  for(i = 0; i < length; i++) {
    data[i] = I2C0->D;
    
    /* Wait for 200 us */
    delay_mc(1);
    
    if(i == (length-1)) {
      /* Send the NACK signal */
      I2C0->C1 |= I2C_C1_TXAK_MASK;
    }
  }
  
  /* Generate a STOP signal */
  I2C0->C1 &= ~I2C_C1_MST_MASK;
  
  return 0;
}
