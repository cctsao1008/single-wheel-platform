#ifndef SWP_BOARD_MOTOR_H
#define SWP_BOARD_MOTOR_H

#include <stdbool.h>

#include "control_types.h"

bool board_motor_init(void);
bool board_motor_apply(swp_actuator_id_t id,
                       const swp_actuator_command_t *command);
void board_motor_safe_off(void);

#endif
