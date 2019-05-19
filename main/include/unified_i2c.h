#pragma once

#include "driver/i2c.h"

#define I2C_WRITE 0
#define I2C_READ 1
#define I2C_CHECK_ACK 0
#define I2C_ACK_VAL 0x0
#define I2C_NACK_VAL 0x1

int i2c_read(i2c_port_t port, int8_t address, int count, uint8_t buf[]);
int i2c_read_reg(i2c_port_t port, int8_t address, uint8_t * value);
int i2c_write(i2c_port_t port, int8_t address, int count, uint8_t buf[]);
int i2c_write_reg(i2c_port_t port, int8_t address, uint8_t value);