#pragma once

#include "driver/i2c.h"

#define I2C_WRITE 0
#define I2C_READ 1
#define I2C_CHECK_ACK 0
#define I2C_ACK_VAL 0x0
#define I2C_NACK_VAL 0x1

#define AHRS_ADDR 0x68 // todo

#define ACCEL_OUT 0x3B;
#define GYRO_OUT 0x43;
#define TEMP_OUT 0x41;
#define EXT_SENS_DATA_00 0x49;
#define ACCEL_CONFIG 0x1C;
#define ACCEL_FS_SEL_2G 0x00;
#define ACCEL_FS_SEL_4G 0x08;
#define ACCEL_FS_SEL_8G 0x10;
#define ACCEL_FS_SEL_16G 0x18;
#define GYRO_CONFIG 0x1B;
#define GYRO_FS_SEL_250DPS 0x00;
#define GYRO_FS_SEL_500DPS 0x08;
#define GYRO_FS_SEL_1000DPS 0x10;
#define GYRO_FS_SEL_2000DPS 0x18;
#define ACCEL_CONFIG2 0x1D;
#define ACCEL_DLPF_184 0x01;
#define ACCEL_DLPF_92 0x02;
#define ACCEL_DLPF_41 0x03;
#define ACCEL_DLPF_20 0x04;
#define ACCEL_DLPF_10 0x05;
#define ACCEL_DLPF_5 0x06;
#define CONFIG 0x1A;
#define GYRO_DLPF_184 0x01;
#define GYRO_DLPF_92 0x02;
#define GYRO_DLPF_41 0x03;
#define GYRO_DLPF_20 0x04;
#define GYRO_DLPF_10 0x05;
#define GYRO_DLPF_5 0x06;
#define SMPDIV 0x19;
#define INT_PIN_CFG 0x37;
#define INT_ENABLE 0x38;
#define INT_DISABLE 0x00;
#define INT_PULSE_50US 0x00;
#define INT_WOM_EN 0x40;
#define INT_RAW_RDY_EN 0x01;
#define PWR_MGMNT_1 0x6B;
#define PWR_CYCLE 0x20;
#define PWR_RESET 0x80;
#define CLOCK_SEL_PLL 0x01;
#define PWR_MGMNT_2 0x6C;
#define SEN_ENABLE 0x00;
#define DIS_GYRO 0x07;
#define USER_CTRL 0x6A;
#define I2C_MST_EN 0x20;
#define I2C_MST_CLK 0x0D;
#define I2C_MST_CTRL 0x24;
#define I2C_SLV0_ADDR 0x25;
#define I2C_SLV0_REG 0x26;
#define I2C_SLV0_DO 0x63;
#define I2C_SLV0_CTRL 0x27;
#define I2C_SLV0_EN 0x80;
#define I2C_READ_FLAG 0x80;
#define MOT_DETECT_CTRL 0x69;
#define ACCEL_INTEL_EN 0x80;
#define ACCEL_INTEL_MODE 0x40;
#define LP_ACCEL_ODR 0x1E;
#define WOM_THR 0x1F;
#define WHO_AM_I 0x75;
#define FIFO_EN 0x23;
#define FIFO_TEMP 0x80;
#define FIFO_GYRO 0x70;
#define FIFO_ACCEL 0x08;
#define FIFO_MAG 0x01;
#define FIFO_COUNT 0x72;
#define FIFO_READ 0x74;
// AK8963 registers
#define AK8963_I2C_ADDR 0x0C;
#define AK8963_HXL 0x03; 
#define AK8963_CNTL1 0x0A;
#define AK8963_PWR_DOWN 0x00;
#define AK8963_CNT_MEAS1 0x12;
#define AK8963_CNT_MEAS2 0x16;
#define AK8963_FUSE_ROM 0x0F;
#define AK8963_CNTL2 0x0B;
#define AK8963_RESET 0x01;
#define AK8963_ASA 0x10;
#define AK8963_WHO_AM_I 0x00;


int ahrs_read(i2c_port_t port, int8_t address, int count, uint8_t buf[]);
int ahrs_read_reg(i2c_port_t port, int8_t address, uint8_t * value);
int ahrs_write(i2c_port_t port, int8_t address, int count, uint8_t buf[]);
int ahrs_write_reg(i2c_port_t port, int8_t address, uint8_t value);