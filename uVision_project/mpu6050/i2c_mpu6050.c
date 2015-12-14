#include "i2c_mpu6050.h"

/**
 * @brief Initialises the I2C interface
 */
void i2c_init(void) {
  /* Enable clock to the pins used as SCL(PTE19) and SDA(PTE18) */
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  
  /* Enable clock to I2C => I2C0 is driven by the Bus Clock = 24 MHz*/
  SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK;
  
  /* Set MUX configuration of PTE18 and PTE19 to I2C */
  PORTE->PCR[18] |= PORT_PCR_MUX(4); /* SDA */
  PORTE->PCR[19] |= PORT_PCR_MUX(4); /* SCL */
  
  /* Minimum "SCL start hold time" is 0.6 us
   * Minimum "SCL stop hold time" is 0.6 us.
   * Minimum "SDA hold time" is 0 us
   * The resulting Baud Rate is 100 kBd/s
   */
  I2C0->F |= I2C_F_MULT(0); // mul = 1
  I2C0->F |= I2C_F_ICR(0x1F); // SCL divider = 240
 
  /* Enable I2C module */
  I2C0->C1 |= I2C_C1_IICEN_MASK;
}

/**
 * @brief Reads an 8-bit register from an I2C slave 
 */
uint8_t i2c_read_register(register uint8_t slaveId, register uint8_t registerAddress){
  /* loop while the bus is still busy */
	i2c_wait_while_busy();
	
	/* send I2C start signal and set write direction, also enables ACK */
	i2c_send_start();
	
	/* send the slave address and wait for the I2C bus operation to complete */
	i2c_send_byte(I2C_WRITE_ADDRESS(slaveId));
	
	/* send the register address */
	i2c_send_byte(registerAddress);
	
	/* signal a repeated start condition */
	i2c_send_repeated_start();

	/* send the read address */
	i2c_send_byte(I2C_READ_ADDRESS(slaveId));
	
	/* switch to receive mode but disable ACK because only one data byte will be read */
	i2c_enter_receive_mode_without_ack();
	
	/* read a dummy byte to drive the clock */
	i2c_read_dummy_byte();
	
	/* stop signal */
	i2c_send_stop();
	
	/* fetch the last received byte */
	register uint8_t result = I2C0->D;
	return result;
}

/**
 * @brief Reads multiple 8-bit registers from an I2C slave
 * @param[in] slaveId The slave device ID
 * @param[in] startRegisterAddress The first register address
 * @param[in] registerCount The number of registers to read; Must be greater than or equal to two.
 * @param[out] buffer The buffer to write into
 * @return 0 is successful, 1 if not
 */
static int i2c_read_registers_internal(register uint8_t slaveId, register uint8_t startRegisterAddress, register uint8_t registerCount, uint8_t *const buffer){
  if(registerCount < 2)
    return -1;
	
	/* loop while the bus is still busy */
	i2c_wait_while_busy();
	
	/* send I2C start signal and set write direction, also enables ACK */
	i2c_send_start();
	
	/* send the slave address and wait for the I2C bus operation to complete */
	i2c_send_byte(I2C_WRITE_ADDRESS(slaveId));
	
	/* send the register address */
	i2c_send_byte(startRegisterAddress);
	
	/* signal a repeated start condition */
	i2c_send_repeated_start();

	/* send the read address */
	i2c_send_byte(I2C_READ_ADDRESS(slaveId));
	
	/* switch to receive mode and assume more than one register */
	i2c_enter_receive_mode_with_ack();
	
	/* read a dummy byte to drive the clock */
	i2c_read_dummy_byte();
	
	/* for all remaining bytes, read */
	--registerCount;
	uint8_t index = 0;
	while (--registerCount > 0)
	{
		/* fetch and store value */
		register uint8_t value = I2C0->D;
		buffer[index++] = value;
		
		/* wait for completion */
		i2c_wait();
	}
	
	/* disable ACK and read second-to-last byte */
	i2c_disable_ack();
	
	/* fetch and store value */
	buffer[index++] = I2C0->D;
	
	/* wait for completion */
	i2c_wait();
	
	/* stop signal */
	i2c_send_stop();
	
	/* fetch the last received byte */
	buffer[index++] = I2C0->D; 
  
  return 0;
}

/**
 * @brief Reads multiple 8-bit registers from an I2C slave
 * @param[in] slaveId The slave device ID
 * @param[in] startRegisterAddress The first register address
 * @param[in] registerCount The number of registers to read; Must be larger than zero.
 * @param[out] buffer The buffer to write into
 * @return 0 is successful, 1 if not
 */
int i2c_read_registers(register uint8_t slaveId, register uint8_t startRegisterAddress, register uint8_t registerCount, register uint8_t *buffer){
	if(registerCount < 1)
    return -1;
	
	if (registerCount >= 2){
		i2c_read_registers_internal(slaveId, startRegisterAddress, registerCount, buffer);
	}else{
		if(registerCount != 1)
      return -1;
		register uint8_t result = i2c_read_register(slaveId, startRegisterAddress);
		buffer[0] = result;
	}
  
  return 0;
}

/**
 * @brief Reads an 8-bit register from an I2C slave 
 */
void i2c_write_register(register uint8_t slaveId, register uint8_t registerAddress, register uint8_t value){
	/* loop while the bus is still busy */
	i2c_wait_while_busy();	
	
	/* send I2C start signal and set write direction*/
	i2c_send_start();
		
	/* send the slave address and wait for the I2C bus operation to complete */
	i2c_send_byte(I2C_WRITE_ADDRESS(slaveId));
	
	/* send the register address */
	i2c_send_byte(registerAddress);
		
	/* send the register value */
	i2c_send_byte(value);
	
	/* issue stop signal by clearing master mode. */
	i2c_send_stop();
}

/**
 * @brief Writes an 8-bit values to an 8-bit registers on an I2C slave starting from registerAddress
 * @param[in] slaveId The device's I2C slave id
 * @param[in] registerAddress Address of the device register to write to
 * @param[in] buffer The buffer with values to write
 * @return 0 is successful, 1 if not
 */
int i2c_write_registers(register uint8_t slaveId, register uint8_t registerAddress, register uint8_t registerCount, register uint8_t *buffer){
  uint8_t i = 0;
  
  /* loop while the bus is still busy */
	i2c_wait_while_busy();	
	
	/* send I2C start signal and set write direction*/
	i2c_send_start();
		
	/* send the slave address and wait for the I2C bus operation to complete */
	i2c_send_byte(I2C_WRITE_ADDRESS(slaveId));
	
	/* send the register address */
	i2c_send_byte(registerAddress);
		
  for(i = 0; i < registerCount; i++) {
    /* send the register value */
    i2c_send_byte(buffer[i]);
  }
	
	/* issue stop signal by clearing master mode. */
	i2c_send_stop();
  
  return 0;
}
