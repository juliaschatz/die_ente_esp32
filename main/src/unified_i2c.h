#pragma once

#include "driver/i2c.h"

#define I2C_WRITE 0
#define I2C_READ 1
#define I2C_CHECK_ACK 0
#define I2C_ACK_VAL I2C_MASTER_ACK
#define I2C_NACK_VAL I2C_MASTER_NACK

int i2c_read(i2c_port_t port, uint8_t device_addr, uint8_t reg_address, int count, uint8_t buf[]);
int i2c_read_reg(i2c_port_t port, uint8_t device_addr, uint8_t reg_address, uint8_t * value);
int i2c_write(i2c_port_t port, uint8_t device_addr, uint8_t reg_address, int count, uint8_t buf[]);
int i2c_write_reg(i2c_port_t port, uint8_t device_addr, uint8_t reg_address, uint8_t value);