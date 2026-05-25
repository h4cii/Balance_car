#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdint.h>

void soft_i2c_init(void);
uint8_t soft_i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
uint8_t soft_i2c_read_regs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
uint8_t soft_i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

#endif
