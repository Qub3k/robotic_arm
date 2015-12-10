#ifndef I2C_MPU6050_H
#define I2C_MPU6050_H

#include "MKL46Z4.h"
#include "extra.h"

void i2c_init(void);
int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

#endif
