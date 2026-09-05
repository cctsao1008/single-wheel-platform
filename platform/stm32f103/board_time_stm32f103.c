#include "board_time.h"

#include "stm32f103_regs.h"

#define STM32F103_CORE_CLOCK_HZ 72000000u
#define STM32F103_CYCLES_PER_US (STM32F103_CORE_CLOCK_HZ / 1000000u)

static bool time_initialized;
static uint32_t last_cycles;
static uint32_t elapsed_us;
static uint32_t cycle_remainder;

bool board_time_init(void)
{
    STM32_DEMCR |= STM32_DEMCR_TRCENA;
    STM32_DWT_CYCCNT = 0u;
    STM32_DWT_CTRL |= STM32_DWT_CTRL_CYCCNTENA;
    time_initialized = (STM32_DWT_CTRL & STM32_DWT_CTRL_CYCCNTENA) != 0u;
    last_cycles = 0u;
    elapsed_us = 0u;
    cycle_remainder = 0u;
    return time_initialized;
}

uint32_t board_time_us(void)
{
    uint32_t now;
    uint32_t delta_cycles;
    uint64_t cycles;

    if (!time_initialized)
        (void)board_time_init();

    now = STM32_DWT_CYCCNT;
    delta_cycles = now - last_cycles;
    last_cycles = now;

    cycles = (uint64_t)cycle_remainder + (uint64_t)delta_cycles;
    elapsed_us += (uint32_t)(cycles / STM32F103_CYCLES_PER_US);
    cycle_remainder = (uint32_t)(cycles % STM32F103_CYCLES_PER_US);
    return elapsed_us;
}

void board_delay_us(uint32_t delay_us)
{
    uint32_t start;
    uint32_t target_cycles;

    if (!time_initialized)
        (void)board_time_init();

    start = STM32_DWT_CYCCNT;
    target_cycles = delay_us * STM32F103_CYCLES_PER_US;
    while ((uint32_t)(STM32_DWT_CYCCNT - start) < target_cycles)
    {
    }
}

void board_delay_ms(uint32_t delay_ms)
{
    while (delay_ms-- != 0u)
        board_delay_us(1000u);
}
