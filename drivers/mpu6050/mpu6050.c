#include "mpu6050.h"
#include "mpu6050_registers.h"

static bool write_u8(mpu6050_t *device, uint8_t reg, uint8_t value)
{
    return device->transport.write_reg(device->transport.context,
                                       device->address_7bit,
                                       reg,
                                       &value,
                                       1u);
}

static bool read_u8(mpu6050_t *device, uint8_t reg, uint8_t *value)
{
    return device->transport.read_reg(device->transport.context,
                                      device->address_7bit,
                                      reg,
                                      value,
                                      1u);
}

static int16_t read_be_i16(const uint8_t *data)
{
    const uint16_t value = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    return (int16_t)value;
}

bool mpu6050_init(mpu6050_t *device,
                  const mpu6050_transport_t *transport,
                  uint8_t address_7bit,
                  const mpu6050_config_t *config)
{
    if ((device == NULL) || (transport == NULL) || (config == NULL) ||
        (transport->read_reg == NULL) || (transport->write_reg == NULL))
    {
        return false;
    }

    device->transport = *transport;
    device->address_7bit = address_7bit;
    device->initialized = false;

    if (!mpu6050_probe(device))
    {
        return false;
    }

    if (!write_u8(device, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_XGYRO))
    {
        return false;
    }

    if (device->transport.delay_ms != NULL)
    {
        device->transport.delay_ms(device->transport.context, 100u);
    }

    if (!mpu6050_configure(device, config))
    {
        return false;
    }

    device->initialized = true;
    return true;
}

bool mpu6050_probe(mpu6050_t *device)
{
    uint8_t who_am_i = 0u;

    if ((device == NULL) || (device->transport.read_reg == NULL))
    {
        return false;
    }

    if (!read_u8(device, MPU6050_REG_WHO_AM_I, &who_am_i))
    {
        return false;
    }

    return who_am_i == MPU6050_WHO_AM_I_VALUE;
}

bool mpu6050_configure(mpu6050_t *device,
                       const mpu6050_config_t *config)
{
    uint32_t base_rate_hz;
    uint32_t divider_plus_one;
    uint8_t divider;
    uint8_t value;

    if ((device == NULL) || (config == NULL) ||
        (device->transport.write_reg == NULL) ||
        (config->gyro_range > MPU6050_GYRO_RANGE_2000_DPS) ||
        (config->accel_range > MPU6050_ACCEL_RANGE_16_G) ||
        (config->dlpf > MPU6050_DLPF_CFG_6) ||
        (config->sample_rate_hz == 0u))
    {
        return false;
    }

    base_rate_hz = (config->dlpf == MPU6050_DLPF_CFG_0) ? 8000u : 1000u;

    if ((config->sample_rate_hz > base_rate_hz) ||
        ((base_rate_hz % config->sample_rate_hz) != 0u))
    {
        return false;
    }

    divider_plus_one = base_rate_hz / config->sample_rate_hz;
    if ((divider_plus_one == 0u) || (divider_plus_one > 256u))
    {
        return false;
    }
    divider = (uint8_t)(divider_plus_one - 1u);

    if (!write_u8(device, MPU6050_REG_CONFIG, (uint8_t)config->dlpf))
    {
        return false;
    }

    value = (uint8_t)((uint8_t)config->gyro_range << 3);
    if (!write_u8(device, MPU6050_REG_GYRO_CONFIG, value))
    {
        return false;
    }

    value = (uint8_t)((uint8_t)config->accel_range << 3);
    if (!write_u8(device, MPU6050_REG_ACCEL_CONFIG, value))
    {
        return false;
    }

    if (!write_u8(device, MPU6050_REG_SMPLRT_DIV, divider))
    {
        return false;
    }

    value = config->data_ready_interrupt ? MPU6050_INT_DATA_READY : 0u;
    if (!write_u8(device, MPU6050_REG_INT_ENABLE, value))
    {
        return false;
    }

    device->config = *config;
    return true;
}

bool mpu6050_read_raw(mpu6050_t *device,
                      mpu6050_raw_sample_t *sample)
{
    uint8_t data[14];

    if ((device == NULL) || (sample == NULL) || !device->initialized ||
        (device->transport.read_reg == NULL))
    {
        return false;
    }

    if (!device->transport.read_reg(device->transport.context,
                                    device->address_7bit,
                                    MPU6050_REG_ACCEL_XOUT_H,
                                    data,
                                    sizeof(data)))
    {
        return false;
    }

    sample->accel_raw[0] = read_be_i16(&data[0]);
    sample->accel_raw[1] = read_be_i16(&data[2]);
    sample->accel_raw[2] = read_be_i16(&data[4]);
    sample->temperature_raw = read_be_i16(&data[6]);
    sample->gyro_raw[0] = read_be_i16(&data[8]);
    sample->gyro_raw[1] = read_be_i16(&data[10]);
    sample->gyro_raw[2] = read_be_i16(&data[12]);
    sample->timestamp_us = (device->transport.get_time_us != NULL)
                               ? device->transport.get_time_us(device->transport.context)
                               : 0u;

    return true;
}

float mpu6050_gyro_lsb_per_dps(mpu6050_gyro_range_t range)
{
    static const float scale[] = {131.0f, 65.5f, 32.8f, 16.4f};

    if (range > MPU6050_GYRO_RANGE_2000_DPS)
    {
        return 0.0f;
    }

    return scale[(unsigned int)range];
}

float mpu6050_accel_lsb_per_g(mpu6050_accel_range_t range)
{
    static const float scale[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};

    if (range > MPU6050_ACCEL_RANGE_16_G)
    {
        return 0.0f;
    }

    return scale[(unsigned int)range];
}
