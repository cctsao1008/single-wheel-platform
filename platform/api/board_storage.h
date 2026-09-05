#ifndef SWP_BOARD_STORAGE_H
#define SWP_BOARD_STORAGE_H

#include <stdbool.h>
#include <stddef.h>

bool board_storage_read(void *dst, size_t size);
bool board_storage_write(const void *src, size_t size);

#endif
