#ifndef SWP_BOARD_UART_H
#define SWP_BOARD_UART_H

#include <stddef.h>
#include <stdint.h>

size_t board_uart_write(const uint8_t *data, size_t size);
size_t board_uart_read(uint8_t *data, size_t capacity);

#endif
