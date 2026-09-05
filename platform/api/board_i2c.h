#ifndef SWP_BOARD_I2C_H
#define SWP_BOARD_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    /* Logical board bus for the on-board MPU6050. This name does not imply
       use of the STM32 I2C1 peripheral. */
    BOARD_I2C_BUS_IMU = 0
} board_i2c_bus_t;

bool board_i2c_init(board_i2c_bus_t bus);
bool board_i2c_read_reg(board_i2c_bus_t bus,
                        uint8_t address_7bit,
                        uint8_t reg,
                        uint8_t *data,
                        size_t length);
bool board_i2c_write_reg(board_i2c_bus_t bus,
                         uint8_t address_7bit,
                         uint8_t reg,
                         const uint8_t *data,
                         size_t length);

#endif
