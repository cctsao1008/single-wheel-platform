#ifndef SWP_STM32F103_REGS_H
#define SWP_STM32F103_REGS_H

#include <stdint.h>

typedef struct
{
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
} stm32_gpio_regs_t;

typedef struct
{
    volatile uint32_t EVCR;
    volatile uint32_t MAPR;
    volatile uint32_t EXTICR[4];
    volatile uint32_t MAPR2;
} stm32_afio_regs_t;

typedef struct
{
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} stm32_exti_regs_t;

typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
} stm32_rcc_regs_t;

typedef struct
{
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SMCR;
    volatile uint32_t DIER;
    volatile uint32_t SR;
    volatile uint32_t EGR;
    volatile uint32_t CCMR1;
    volatile uint32_t CCMR2;
    volatile uint32_t CCER;
    volatile uint32_t CNT;
    volatile uint32_t PSC;
    volatile uint32_t ARR;
    volatile uint32_t RCR;
    volatile uint32_t CCR1;
    volatile uint32_t CCR2;
    volatile uint32_t CCR3;
    volatile uint32_t CCR4;
    volatile uint32_t BDTR;
    volatile uint32_t DCR;
    volatile uint32_t DMAR;
} stm32_tim_regs_t;

#define STM32_AFIO  ((stm32_afio_regs_t *)0x40010000u)
#define STM32_EXTI  ((stm32_exti_regs_t *)0x40010400u)
#define STM32_GPIOA ((stm32_gpio_regs_t *)0x40010800u)
#define STM32_GPIOB ((stm32_gpio_regs_t *)0x40010C00u)
#define STM32_GPIOC ((stm32_gpio_regs_t *)0x40011000u)
#define STM32_RCC   ((stm32_rcc_regs_t *)0x40021000u)
#define STM32_TIM2  ((stm32_tim_regs_t *)0x40000000u)
#define STM32_TIM3  ((stm32_tim_regs_t *)0x40000400u)
#define STM32_TIM4  ((stm32_tim_regs_t *)0x40000800u)

#define STM32_DEMCR      (*(volatile uint32_t *)0xE000EDFCu)
#define STM32_DWT_CTRL   (*(volatile uint32_t *)0xE0001000u)
#define STM32_DWT_CYCCNT (*(volatile uint32_t *)0xE0001004u)
#define STM32_NVIC_ISER  ((volatile uint32_t *)0xE000E100u)
#define STM32_NVIC_ICER  ((volatile uint32_t *)0xE000E180u)
#define STM32_NVIC_IPR   ((volatile uint8_t *)0xE000E400u)

#define STM32_RCC_APB2_AFIOEN  (1u << 0)
#define STM32_RCC_APB2_IOPAEN  (1u << 2)
#define STM32_RCC_APB2_IOPBEN  (1u << 3)
#define STM32_RCC_APB2_IOPCEN  (1u << 4)
#define STM32_RCC_APB1_TIM2EN  (1u << 0)
#define STM32_RCC_APB1_TIM3EN  (1u << 1)
#define STM32_RCC_APB1_TIM4EN  (1u << 2)

#define STM32_TIM_CR1_CEN      (1u << 0)
#define STM32_TIM_CR1_ARPE     (1u << 7)
#define STM32_TIM_EGR_UG       (1u << 0)
#define STM32_TIM_SMCR_SMS_ENCODER3 3u

#define STM32_DEMCR_TRCENA       (1u << 24)
#define STM32_DWT_CTRL_CYCCNTENA (1u << 0)

static inline void stm32_gpio_config_nibble(stm32_gpio_regs_t *gpio,
                                             unsigned int pin,
                                             uint32_t nibble)
{
    volatile uint32_t *reg;
    uint32_t shift;
    uint32_t value;

    if (pin < 8u)
    {
        reg = &gpio->CRL;
        shift = pin * 4u;
    }
    else
    {
        reg = &gpio->CRH;
        shift = (pin - 8u) * 4u;
    }

    value = *reg;
    value &= ~(0xFu << shift);
    value |= (nibble & 0xFu) << shift;
    *reg = value;
}

static inline void stm32_gpio_write(stm32_gpio_regs_t *gpio,
                                    unsigned int pin,
                                    int high)
{
    if (high != 0)
        gpio->BSRR = 1u << pin;
    else
        gpio->BRR = 1u << pin;
}

static inline int stm32_gpio_read(const stm32_gpio_regs_t *gpio,
                                  unsigned int pin)
{
    return ((gpio->IDR >> pin) & 1u) != 0u;
}

static inline void stm32_nvic_enable_irq(unsigned int irq_number)
{
    STM32_NVIC_ISER[irq_number / 32u] = 1u << (irq_number % 32u);
}

static inline void stm32_nvic_disable_irq(unsigned int irq_number)
{
    STM32_NVIC_ICER[irq_number / 32u] = 1u << (irq_number % 32u);
}

#endif
