#ifndef SWP_CONTROL_TYPES_H
#define SWP_CONTROL_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t timestamp_us;

    float roll_rad;
    float roll_rate_rad_s;

    float pitch_rad;
    float pitch_rate_rad_s;

    float reaction_wheel_speed_rad_s;
    float drive_wheel_speed_rad_s;
    float yaw_rate_rad_s;

    float battery_v;
    bool valid;
} swp_robot_state_t;

typedef struct
{
    float roll_effort;
    float pitch_effort;
    float yaw_effort;
    bool valid;
} swp_control_effort_t;

typedef enum
{
    SWP_ACTUATOR_REACTION_WHEEL = 0,
    SWP_ACTUATOR_DRIVE_WHEEL,
    SWP_ACTUATOR_SPIN
} swp_actuator_id_t;

typedef struct
{
    float normalized_command; /* -1.0 .. +1.0 */
    bool enable;
    bool brake;
    bool valid;
} swp_actuator_command_t;

#endif
