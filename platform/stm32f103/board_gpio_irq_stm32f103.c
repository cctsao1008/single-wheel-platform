#include "board_gpio_irq.h"

#include <stddef.h>

#include "stm32f103_regs.h"

#define MPU_INT_PIN 13u
#define EXTI15_10_IRQn 40u

static board_gpio_irq_callback_t imu_callback;
static void *imu_callback_context;
static bool gpio_irq_initialized;

bool board_gpio_irq_init(void)
{
    STM32_RCC->APB2ENR |= STM32_RCC_APB2_AFIOEN | STM32_RCC_APB2_IOPCEN;

    /* PC13 = MPU_INT, floating input. */
    stm32_gpio_config_nibble(STM32_GPIOC, MPU_INT_PIN, 0x4u);

    /* EXTI13 source = GPIOC. EXTICR4 bits [7:4]. */
    STM32_AFIO->EXTICR[3] &= ~(0xFu << 4);
    STM32_AFIO->EXTICR[3] |=  (0x2u << 4);

    STM32_EXTI->IMR &= ~(1u << MPU_INT_PIN);
    STM32_EXTI->RTSR &= ~(1u << MPU_INT_PIN);
    STM32_EXTI->FTSR &= ~(1u << MPU_INT_PIN);
    STM32_EXTI->PR = 1u << MPU_INT_PIN;

    STM32_NVIC_IPR[EXTI15_10_IRQn] = 0x80u;
    stm32_nvic_enable_irq(EXTI15_10_IRQn);
    gpio_irq_initialized = true;
    return true;
}

bool board_gpio_irq_register(board_gpio_irq_source_t source,
                             board_gpio_irq_edge_t edge,
                             board_gpio_irq_callback_t callback,
                             void *context)
{
    if ((source != BOARD_GPIO_IRQ_IMU_INT) || (callback == NULL))
        return false;

    if (!gpio_irq_initialized && !board_gpio_irq_init())
        return false;

    STM32_EXTI->IMR &= ~(1u << MPU_INT_PIN);
    STM32_EXTI->RTSR &= ~(1u << MPU_INT_PIN);
    STM32_EXTI->FTSR &= ~(1u << MPU_INT_PIN);

    switch (edge)
    {
        case BOARD_GPIO_IRQ_EDGE_RISING:
            STM32_EXTI->RTSR |= 1u << MPU_INT_PIN;
            break;
        case BOARD_GPIO_IRQ_EDGE_FALLING:
            STM32_EXTI->FTSR |= 1u << MPU_INT_PIN;
            break;
        case BOARD_GPIO_IRQ_EDGE_BOTH:
            STM32_EXTI->RTSR |= 1u << MPU_INT_PIN;
            STM32_EXTI->FTSR |= 1u << MPU_INT_PIN;
            break;
        default:
            return false;
    }

    imu_callback = callback;
    imu_callback_context = context;
    STM32_EXTI->PR = 1u << MPU_INT_PIN;
    STM32_EXTI->IMR |= 1u << MPU_INT_PIN;
    return true;
}

void board_gpio_irq_enable(board_gpio_irq_source_t source, bool enable)
{
    if (source != BOARD_GPIO_IRQ_IMU_INT)
        return;

    if (enable)
        STM32_EXTI->IMR |= 1u << MPU_INT_PIN;
    else
        STM32_EXTI->IMR &= ~(1u << MPU_INT_PIN);
}

void EXTI15_10_IRQHandler(void)
{
    if ((STM32_EXTI->PR & (1u << MPU_INT_PIN)) != 0u)
    {
        STM32_EXTI->PR = 1u << MPU_INT_PIN;
        if (imu_callback != NULL)
            imu_callback(imu_callback_context);
    }
}
