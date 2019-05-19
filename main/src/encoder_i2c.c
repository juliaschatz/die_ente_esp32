#include "encoder_i2c.h"


int encoder_read(i2c_port_t port, int8_t address, int count, uint8_t buf[]) {
    if (count == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // Start read transaction, send address byte
    i2c_master_write_byte(cmd, (ENCODER_ADDR << 1) | I2C_WRITE, I2C_CHECK_ACK);
    // Write word byte
    i2c_master_write_byte(cmd, address, I2C_CHECK_ACK);
    i2c_master_start(cmd);
    // Send address byte again
    i2c_master_write_byte(cmd, (ENCODER_ADDR << 1) | I2C_READ, I2C_CHECK_ACK);
    // Actually do read
    if (count > 1) {
        i2c_master_read(cmd, buf, count - 1, I2C_ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + count - 1, I2C_NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int encoder_write(i2c_port_t port, int8_t address, int count, uint8_t buf[]) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // Begin write transaction
    i2c_master_write_byte(cmd, (ENCODER_ADDR << 1) | I2C_WRITE, I2C_CHECK_ACK);
    // Set address
    i2c_master_write_byte(cmd, address, I2C_CHECK_ACK);
    // Write
    i2c_master_write(cmd, buf, count, I2C_CHECK_ACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void encoder_configure(i2c_port_t port, encoder_conf settings);

float encoder_read_angle(i2c_port_t port) {
    uint8_t buf[2];
    encoder_read(port, ENCODER_REGISTER_ANGLE, 2, buf);
    short angle_int = (buf[0] << 8) | buf[1];
    return 2 * M_PI * angle_int / 4096;
}

int encoder_read_status(i2c_port_t port) {
    uint8_t buf[1];
    buf[0] = 0;
    encoder_read(port, ENCODER_REGISTER_STATUS, 1, buf);
    if ((buf[0] >> 3) & 1) { // MH
        return ENCODER_MAGNET_CLOSE;
    }
    else if ((buf[0] >> 4) & 1) { // ML
        return ENCODER_MAGNET_FAR;
    }
    else if ((buf[0] >> 5) & 1) { // MD
        return ENCODER_MAGNET_OK;
    }
    return -1;
}

int encoder_read_agc(i2c_port_t port) {
    uint8_t buf[1];
    buf[0] = 0;
    encoder_read(port, ENCODER_REGISTER_AGC, 1, buf);
    return buf[0];
}