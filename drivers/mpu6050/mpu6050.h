#ifndef SWP_MPU6050_H
#define SWP_MPU6050_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef bool (*mpu6050_read_reg_fn)(void *context,
                                    uint8_t address_7bit,
                                    uint8_t reg,
                                    uint8_t *data,
                                    size_t length);
typedef bool (*mpu6050_write_reg_fn)(void *context,
                                     uint8_t address_7bit,
                                     uint8_t reg,
                                     const uint8_t *data,
                                     size_t length);
typedef void (*mpu6050_delay_ms_fn)(void *context, uint32_t delay_ms);
typedef uint64_t (*mpu6050_get_time_us_fn)(void *context);

typedef struct
{
    mpu6050_read_reg_fn read_reg;
    mpu6050_write_reg_fn write_reg;
    mpu6050_delay_ms_fn delay_ms;
    mpu6050_get_time_us_fn get_time_us;
    void *context;
} mpu6050_transport_t;

typedef enum
{
    MPU6050_GYRO_RANGE_250_DPS = 0,
    MPU6050_GYRO_RANGE_500_DPS,
    MPU6050_GYRO_RANGE_1000_DPS,
    MPU6050_GYRO_RANGE_2000_DPS
} mpu6050_gyro_range_t;

typedef enum
{
    MPU6050_ACCEL_RANGE_2_G = 0,
    MPU6050_ACCEL_RANGE_4_G,
    MPU6050_ACCEL_RANGE_8_G,
    MPU6050_ACCEL_RANGE_16_G
} mpu6050_accel_range_t;

typedef enum
{
    MPU6050_DLPF_CFG_0 = 0,
    MPU6050_DLPF_CFG_1,
    MPU6050_DLPF_CFG_2,
    MPU6050_DLPF_CFG_3,
    MPU6050_DLPF_CFG_4,
    MPU6050_DLPF_CFG_5,
    MPU6050_DLPF_CFG_6
} mpu6050_dlpf_t;

typedef struct
{
    mpu6050_gyro_range_t gyro_range;
    mpu6050_accel_range_t accel_range;
    mpu6050_dlpf_t dlpf;
    uint16_t sample_rate_hz;
    bool data_ready_interrupt;
} mpu6050_config_t;

typedef struct
{
    uint64_t timestamp_us;
    int16_t accel_raw[3];
    int16_t temperature_raw;
    int16_t gyro_raw[3];
} mpu6050_raw_sample_t;

typedef struct
{
    mpu6050_transport_t transport;
    mpu6050_config_t config;
    uint8_t address_7bit;
    bool initialized;
} mpu6050_t;

bool mpu6050_init(mpu6050_t *device,
                  const mpu6050_transport_t *transport,
                  uint8_t address_7bit,
                  const mpu6050_config_t *config);
bool mpu6050_probe(mpu6050_t *device);
bool mpu6050_configure(mpu6050_t *device,
                       const mpu6050_config_t *config);
bool mpu6050_read_raw(mpu6050_t *device,
                      mpu6050_raw_sample_t *sample);

float mpu6050_gyro_lsb_per_dps(mpu6050_gyro_range_t range);
float mpu6050_accel_lsb_per_g(mpu6050_accel_range_t range);

#endif
