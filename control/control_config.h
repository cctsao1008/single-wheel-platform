#ifndef SWP_CONTROL_CONFIG_H
#define SWP_CONTROL_CONFIG_H

typedef struct
{
    float equilibrium_rad;

    float attitude_kp;
    float attitude_ki;
    float attitude_kd;

    float speed_kp;
    float speed_ki;
    float speed_kd;

    float integrator_min;
    float integrator_max;

    float output_min;
    float output_max;
} swp_axis_controller_config_t;

typedef struct
{
    swp_axis_controller_config_t roll;
    swp_axis_controller_config_t pitch;

    float control_period_s;
    float complementary_accel_weight;
} swp_control_config_t;

#endif
