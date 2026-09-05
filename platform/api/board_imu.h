#ifndef SWP_BOARD_IMU_H
#define SWP_BOARD_IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t timestamp_us;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    bool valid;
} board_imu_sample_t;

bool board_imu_init(void);
bool board_imu_read(board_imu_sample_t *sample);

#endif
