#ifndef SWP_BOARD_GPIO_IRQ_H
#define SWP_BOARD_GPIO_IRQ_H

#include <stdbool.h>

typedef enum
{
    BOARD_GPIO_IRQ_IMU_INT = 0
} board_gpio_irq_source_t;

typedef enum
{
    BOARD_GPIO_IRQ_EDGE_RISING = 0,
    BOARD_GPIO_IRQ_EDGE_FALLING,
    BOARD_GPIO_IRQ_EDGE_BOTH
} board_gpio_irq_edge_t;

typedef void (*board_gpio_irq_callback_t)(void *context);

bool board_gpio_irq_init(void);
bool board_gpio_irq_register(board_gpio_irq_source_t source,
                             board_gpio_irq_edge_t edge,
                             board_gpio_irq_callback_t callback,
                             void *context);
void board_gpio_irq_enable(board_gpio_irq_source_t source, bool enable);

#endif
