#include "board_encoder.h"

#include <stddef.h>

#include "stm32f103_regs.h"

static bool encoder_initialized;

static void encoder_timer_init(stm32_tim_regs_t *tim)
{
    tim->CR1 = 0u;
    tim->PSC = 0u;
    tim->ARR = 0xFFFFu;
    tim->CCMR1 = (1u << 0) | (3u << 4) |
                  (1u << 8) | (3u << 12);
    tim->CCER = 0u;
    tim->SMCR = STM32_TIM_SMCR_SMS_ENCODER3;
    tim->CNT = 0u;
    tim->EGR = STM32_TIM_EGR_UG;
    tim->CR1 = STM32_TIM_CR1_CEN;
}

bool board_encoder_init(void)
{
    STM32_RCC->APB2ENR |= STM32_RCC_APB2_IOPAEN | STM32_RCC_APB2_IOPBEN;
    STM32_RCC->APB1ENR |= STM32_RCC_APB1_TIM2EN | STM32_RCC_APB1_TIM4EN;

    /* Encoder 1: PA0/PA1 -> TIM2_CH1/CH2. */
    stm32_gpio_config_nibble(STM32_GPIOA, 0u, 0x4u);
    stm32_gpio_config_nibble(STM32_GPIOA, 1u, 0x4u);

    /* Encoder 2: PB6/PB7 -> TIM4_CH1/CH2. */
    stm32_gpio_config_nibble(STM32_GPIOB, 6u, 0x4u);
    stm32_gpio_config_nibble(STM32_GPIOB, 7u, 0x4u);

    encoder_timer_init(STM32_TIM2);
    encoder_timer_init(STM32_TIM4);
    encoder_initialized = true;
    return true;
}

bool board_encoder_read_delta(board_encoder_id_t id, int32_t *delta)
{
    stm32_tim_regs_t *tim;
    int16_t raw;

    if (delta == NULL)
        return false;
    if (!encoder_initialized && !board_encoder_init())
        return false;

    switch (id)
    {
        case BOARD_ENCODER_REACTION:
            tim = STM32_TIM2;
            break;
        case BOARD_ENCODER_DRIVE:
            tim = STM32_TIM4;
            break;
        case BOARD_ENCODER_SPIN:
        default:
            return false;
    }

    raw = (int16_t)(uint16_t)tim->CNT;
    tim->CNT = 0u;
    *delta = (int32_t)raw;
    return true;
}
