#ifndef SWP_BOARD_MOTOR_H
#define SWP_BOARD_MOTOR_H

#include <stdbool.h>

typedef enum
{
    BOARD_MOTOR_REACTION_WHEEL = 0,
    BOARD_MOTOR_DRIVE_WHEEL,
    BOARD_MOTOR_SPIN
} board_motor_id_t;

typedef struct
{
    float normalized_command; /* -1.0 .. +1.0 */
    bool enable;
    bool brake;
} board_motor_command_t;

bool board_motor_init(void);
bool board_motor_apply(board_motor_id_t id,
                       const board_motor_command_t *command);
void board_motor_safe_off(void);

#endif
