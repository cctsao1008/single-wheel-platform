#ifndef SWP_BOARD_ENCODER_H
#define SWP_BOARD_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BOARD_ENCODER_REACTION = 0,
    BOARD_ENCODER_DRIVE,
    BOARD_ENCODER_SPIN
} board_encoder_id_t;

bool board_encoder_init(void);
bool board_encoder_read_delta(board_encoder_id_t id, int32_t *delta);

#endif
