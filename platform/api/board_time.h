#ifndef SWP_BOARD_TIME_H
#define SWP_BOARD_TIME_H

#include <stdbool.h>
#include <stdint.h>

bool board_time_init(void);
uint32_t board_time_us(void);
void board_delay_us(uint32_t delay_us);
void board_delay_ms(uint32_t delay_ms);

#endif
