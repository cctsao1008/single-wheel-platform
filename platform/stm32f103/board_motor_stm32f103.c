#include "board_motor.h"

#include <stddef.h>

#include "stm32f103_regs.h"

#define PWM_PERIOD_COUNTS 7200u

static bool motor_initialized;

static uint32_t magnitude_to_counts(float command)
{
    float magnitude = command < 0.0f ? -command : command;
    return (uint32_t)(magnitude * (float)PWM_PERIOD_COUNTS + 0.5f);
}

static bool command_is_valid(float command)
{
    return (command >= -1.0f) && (command <= 1.0f);
}

static void reaction_off(void) { STM32_TIM3->CCR4 = PWM_PERIOD_COUNTS; }
static void drive_off(void)    { STM32_TIM3->CCR1 = 0u; }
static void spin_off(void)     { STM32_TIM3->CCR3 = 0u; }

bool board_motor_init(void)
{
    STM32_RCC->APB2ENR |= STM32_RCC_APB2_IOPAEN | STM32_RCC_APB2_IOPBEN;
    STM32_RCC->APB1ENR |= STM32_RCC_APB1_TIM3EN;

    /* PWM: PA6=TIM3_CH1 (drive), PB0=TIM3_CH3 (spin), PB1=TIM3_CH4 (reaction). */
    stm32_gpio_config_nibble(STM32_GPIOA, 6u, 0xBu);
    stm32_gpio_config_nibble(STM32_GPIOB, 0u, 0xBu);
    stm32_gpio_config_nibble(STM32_GPIOB, 1u, 0xBu);

    /* Direction: PA4=drive, PB10=spin, PB11=reaction. */
    stm32_gpio_config_nibble(STM32_GPIOA, 4u, 0x2u);
    stm32_gpio_config_nibble(STM32_GPIOB, 10u, 0x2u);
    stm32_gpio_config_nibble(STM32_GPIOB, 11u, 0x2u);
    stm32_gpio_write(STM32_GPIOA, 4u, 0);
    stm32_gpio_write(STM32_GPIOB, 10u, 1);
    stm32_gpio_write(STM32_GPIOB, 11u, 0);

    STM32_TIM3->CR1 = 0u;
    STM32_TIM3->PSC = 0u;
    STM32_TIM3->ARR = PWM_PERIOD_COUNTS - 1u;

    STM32_TIM3->CCMR1 = (6u << 4) | (1u << 3);
    STM32_TIM3->CCMR2 = (6u << 4) | (1u << 3) |
                         (6u << 12) | (1u << 11);
    STM32_TIM3->CCER = (1u << 0) | (1u << 8) | (1u << 12);

    board_motor_safe_off();
    STM32_TIM3->EGR = STM32_TIM_EGR_UG;
    STM32_TIM3->CR1 = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_CEN;
    motor_initialized = true;
    return true;
}

bool board_motor_apply(board_motor_id_t id,
                       const board_motor_command_t *command)
{
    uint32_t counts;

    if ((command == NULL) || !command_is_valid(command->normalized_command))
        return false;

    if (!motor_initialized && !board_motor_init())
        return false;

    /* Brake polarity is not yet frozen in the board contract. Fail safe. */
    if (command->brake)
    {
        switch (id)
        {
            case BOARD_MOTOR_REACTION_WHEEL: reaction_off(); break;
            case BOARD_MOTOR_DRIVE_WHEEL: drive_off(); break;
            case BOARD_MOTOR_SPIN: spin_off(); break;
            default: return false;
        }
        return false;
    }

    if (!command->enable)
    {
        switch (id)
        {
            case BOARD_MOTOR_REACTION_WHEEL: reaction_off(); return true;
            case BOARD_MOTOR_DRIVE_WHEEL: drive_off(); return true;
            case BOARD_MOTOR_SPIN: spin_off(); return true;
            default: return false;
        }
    }

    counts = magnitude_to_counts(command->normalized_command);

    switch (id)
    {
        case BOARD_MOTOR_REACTION_WHEEL:
            /* BLDC_1 uses the electrically inverted PWM convention of the reference board. */
            stm32_gpio_write(STM32_GPIOB, 11u, command->normalized_command < 0.0f);
            STM32_TIM3->CCR4 = PWM_PERIOD_COUNTS - counts;
            return true;

        case BOARD_MOTOR_DRIVE_WHEEL:
            stm32_gpio_write(STM32_GPIOA, 4u, command->normalized_command < 0.0f);
            STM32_TIM3->CCR1 = counts;
            return true;

        case BOARD_MOTOR_SPIN:
            stm32_gpio_write(STM32_GPIOB, 10u, command->normalized_command >= 0.0f);
            STM32_TIM3->CCR3 = counts;
            return true;

        default:
            return false;
    }
}

void board_motor_safe_off(void)
{
    reaction_off();
    drive_off();
    spin_off();
}
