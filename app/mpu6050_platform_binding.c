#include "mpu6050_platform_binding.h"

#include <stddef.h>

#include "board_i2c.h"
#include "board_time.h"

static bool transport_read(void *context,
                           uint8_t address_7bit,
                           uint8_t reg,
                           uint8_t *data,
                           size_t length)
{
    (void)context;
    return board_i2c_read_reg(BOARD_I2C_BUS_1,
                              address_7bit,
                              reg,
                              data,
                              length);
}

static bool transport_write(void *context,
                            uint8_t address_7bit,
                            uint8_t reg,
                            const uint8_t *data,
                            size_t length)
{
    (void)context;
    return board_i2c_write_reg(BOARD_I2C_BUS_1,
                               address_7bit,
                               reg,
                               data,
                               length);
}

static void transport_delay_ms(void *context, uint32_t delay_ms)
{
    (void)context;
    board_delay_ms(delay_ms);
}

static uint64_t transport_get_time_us(void *context)
{
    static uint32_t last_time;
    static uint64_t epoch;
    uint32_t now;

    (void)context;
    now = board_time_us();
    if (now < last_time)
        epoch += (1ull << 32);
    last_time = now;
    return epoch | (uint64_t)now;
}

bool swp_mpu6050_make_transport(mpu6050_transport_t *transport)
{
    if (transport == NULL)
        return false;

    if (!board_time_init() || !board_i2c_init(BOARD_I2C_BUS_1))
        return false;

    transport->read_reg = transport_read;
    transport->write_reg = transport_write;
    transport->delay_ms = transport_delay_ms;
    transport->get_time_us = transport_get_time_us;
    transport->context = NULL;
    return true;
}
