/**
 * @addtogroup  DRIVERS Sensor Driver Layer
 * @brief       Hardware drivers to communicate with sensors via I2C.
 * @{
 * 		@file i2c_mpu6050.h
 * 		Library used to establish the I2C communiation using the FRDM-MKL46Z board.
 */

#ifndef I2C_MPU6050_H
#define I2C_MPU6050_H

#include "MKL46Z4.h"
#include "extra.h"
#include "uart.h"

/**
 * @brief Helper macro for {@see TOKENPASTE(a,b)} to merge token.
 */
#define TOKENPASTE_HELPER(x, y) x ## y

/**
 * @brief Merges two tokens, expanding any macros.
 */
#define TOKENPASTE(x, y) TOKENPASTE_HELPER(x, y)

/**
 * @brief Macro to mark variables that are unused by intention.
 */
#define INTENTIONALLY_UNUSED(type) type __attribute__((unused)) TOKENPASTE(unused, __COUNTER__)

/**
 * @brief Encodes the read address from the 7-bit slave address.
 */
#define I2C_READ_ADDRESS(slaveAddress) 		((uint8_t)((slaveAddress << 1) | 1))

/**
 * @brief Encodes the write address from the 7-bit slave address.
 */
#define I2C_WRITE_ADDRESS(slaveAddress) 	((uint8_t)((slaveAddress << 1) | 0))

/**
 * @brief Reads an 8-bit register from an I2C slave 
 * @param[in] slaveId The device's I2C slave id
 * @param[in] registerAddress Address of the device register to read
 * @return The value at the register
 */
uint8_t i2c_read_register(register uint8_t slaveId, register uint8_t registerAddress);

/**
 * @brief Reads multiple 8-bit registers from an I2C slave
 * @param[in] slaveId The slave device ID
 * @param[in] startRegisterAddress The first register address
 * @param[in] registerCount The number of registers to read; Must be larger than zero.
 * @param[out] buffer The buffer to write into
 * @return 0 is successful, 1 if not
 */
int i2c_read_registers(register uint8_t slaveId, register uint8_t startRegisterAddress, register uint8_t registerCount, uint8_t *buffer);

/**
 * @brief Writes an 8-bit value to an 8-bit register on an I2C slave
 * @param[in] slaveId The device's I2C slave id
 * @param[in] registerAddress Address of the device register to write to
 * @param[in] value The value to write
 */
void i2c_write_register(register uint8_t slaveId, register uint8_t registerAddress, register uint8_t value);

/**
 * @brief Writes an 8-bit values to an 8-bit registers on an I2C slave starting from registerAddress
 * @param[in] slaveId The device's I2C slave id
 * @param[in] registerAddress Address of the device register to write to
 * @param[in] buffer The buffer with values to write
 * @return 0 is successful, 1 if not
 */
int i2c_write_registers(register uint8_t slaveId, register uint8_t registerAddress, register uint8_t registerCount, register uint8_t *buffer);

/**
 * @brief Initiates a register read after the module was brought into TX mode.
 * @param[in] slaveId The slave id
 * @param[in] registerAddress the register to read from 
 */
void i2c_initiate_register_read_at(const register uint8_t slaveId, const register uint8_t registerAddress);

/**
 * @brief Initializes the I2C interface.
 */
void i2c_init(void);

/**
 * @brief Waits for an I2C bus operation to complete.
 */
static inline void i2c_wait(){
  /** 
  *  loop until interrupt is detected 
  */
	while((I2C0->S & I2C_S_IICIF_MASK)==0) {}	
    
  /**
  *	 clear interrupt flag
  */
	I2C0->S |= I2C_S_IICIF_MASK; 
}

/**
 * @brief Waits for an I2C bus operation to complete.
 */
static inline void i2c_wait_while_busy(){
	while((I2C0->S & I2C_S_BUSY_MASK)!=0) {}
}

/**
 * @brief Sends a start condition and enters TX mode.
 */
static inline void i2c_send_start(){
	I2C0->C1 |= ((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK) | ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
}

/**
 * @brief Enters transmit mode.
 */
static inline void i2c_enter_transmit_mode(){
	I2C0->C1 |= ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
}

/**
 * @brief Enters receive mode.
 */
static inline void i2c_enter_receive_mode(){
	I2C0->C1 &= ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
}

/**
 * @brief Enters receive mode and enables ACK.
 * 
 * Enabling ACK may be required when more than one data byte will be read.
 */
static inline void i2c_enter_receive_mode_with_ack(){
	I2C0->C1 &= ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK) & ~((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
}

/**
 * @brief Enters receive mode and disables ACK.
 * 
 * Disabling ACK may be required when only one data byte will be read.
 */
static inline void i2c_enter_receive_mode_without_ack(){
	/** Straightforward method of clearing TX mode and
	 * setting NACK bit sending.
	 */
	register uint8_t reg = I2C0->C1;
	reg &= ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
	reg |=  ((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
	I2C0->C1 = reg;
}

/**
 * @brief Sends a repeated start condition and enters TX mode.
 */
static inline void i2c_send_repeated_start(){
	I2C0->C1 |= ((1 << I2C_C1_RSTA_SHIFT) & I2C_C1_RSTA_MASK) | ((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
}

/**
 * @brief Sends a stop condition (also leaves TX mode).
 */
static inline void i2c_send_stop(){
	I2C0->C1 &= ~((1 << I2C_C1_MST_SHIFT) & I2C_C1_MST_MASK) & ~((1 << I2C_C1_TX_SHIFT) & I2C_C1_TX_MASK);
}

/**
 * @brief Enables sending of ACK.
 * 
 * Enabling ACK may be required when more than one data byte will be read.
 */
static inline void i2c_enable_ack(){
	I2C0->C1 &= ~((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
}

/**
 * @brief Enables sending of NACK (disabling ACK).
 * 
 * Enabling NACK may be required when no more data byte will be read.
 */
static inline void i2c_disable_ack(){
	I2C0->C1 |= ((1 << I2C_C1_TXAK_SHIFT) & I2C_C1_TXAK_MASK);
}

/**
 * @brief Sends a byte over the I2C bus and waits for the operation to complete
 * @param[in] value The byte to send
 * I2C_SendBlocking().
 */
static inline void i2c_send_byte(const uint8_t value){
	I2C0->D = value;
	i2c_wait();
}

/**
 * @brief Reads a byte over the I2C bus and drives the clock for another byte
 * @return There received byte
 * I2C_ReceiveDriving().
 */
static inline uint8_t i2c_read_byte(){ 
	register uint8_t value = I2C0->D;
	i2c_wait();
	return value;
}

/**
 * @brief Reads a byte over the I2C bus and drives the clock for another byte, while sending NACK
 * @return There received byte
 * I2C_ReceiveDrivingWithNack().
 */
static inline uint8_t I2C_read_byte_with_nack(){ 
	i2c_disable_ack();
	return i2c_read_byte();
}

/**
 * @brief Reads the last byte over the I2C bus and sends a stop condition
 * @return There received byte
 * I2C_ReceiveAndStop().
 */
static inline uint8_t i2c_read_and_stop(){ 
	i2c_send_stop();
	return I2C0->D;
}

/**
 * @brief Reads a byte over the I2C bus and sends a repeated start condition.
 * @return There received byte
 * 
 * The I2C module is in transmit mode afterwards.
 * I2C_ReceiveAndRestart().
 */
static inline uint8_t i2c_read_and_restart(){ 
	i2c_send_repeated_start();
	return I2C0->D;
}

/**
 * @brief Drives the clock in receiver mode in order to receive the first byte.
 *
 *  I2C_ReceiverModeDriveClock().
 */
static inline void i2c_read_dummy_byte(){ 
	INTENTIONALLY_UNUSED(register uint8_t) = I2C0->D;
	i2c_wait();
}
#endif

/**@}*/
