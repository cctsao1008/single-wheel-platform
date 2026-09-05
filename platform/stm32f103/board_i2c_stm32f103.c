#include "board_i2c.h"

#include "board_time.h"
#include "stm32f103_regs.h"

#define I2C_SDA_PIN 8u
#define I2C_SCL_PIN 9u
#define I2C_HALF_PERIOD_US 2u
#define I2C_STRETCH_TIMEOUT_US 100u

static bool i2c_initialized;

static void sda_release(void) { stm32_gpio_write(STM32_GPIOB, I2C_SDA_PIN, 1); }
static void sda_low(void) { stm32_gpio_write(STM32_GPIOB, I2C_SDA_PIN, 0); }
static void scl_release(void) { stm32_gpio_write(STM32_GPIOB, I2C_SCL_PIN, 1); }
static void scl_low(void) { stm32_gpio_write(STM32_GPIOB, I2C_SCL_PIN, 0); }
static int sda_read(void) { return stm32_gpio_read(STM32_GPIOB, I2C_SDA_PIN); }
static int scl_read(void) { return stm32_gpio_read(STM32_GPIOB, I2C_SCL_PIN); }
static void i2c_delay(void) { board_delay_us(I2C_HALF_PERIOD_US); }

static bool wait_scl_high(void)
{
    const uint32_t start = board_time_us();
    scl_release();
    while (!scl_read())
    {
        if ((uint32_t)(board_time_us() - start) >= I2C_STRETCH_TIMEOUT_US)
            return false;
    }
    return true;
}

static bool i2c_start(void)
{
    sda_release();
    if (!wait_scl_high())
        return false;
    i2c_delay();
    if (!sda_read())
        return false;
    sda_low();
    i2c_delay();
    scl_low();
    return true;
}

static void i2c_stop(void)
{
    sda_low();
    i2c_delay();
    if (wait_scl_high())
    {
        i2c_delay();
        sda_release();
        i2c_delay();
    }
    else
    {
        sda_release();
    }
}

static bool i2c_write_byte(uint8_t value)
{
    unsigned int bit;
    bool ack;

    for (bit = 0u; bit < 8u; ++bit)
    {
        if ((value & 0x80u) != 0u)
            sda_release();
        else
            sda_low();

        i2c_delay();
        if (!wait_scl_high())
            return false;
        i2c_delay();
        scl_low();
        value <<= 1;
    }

    sda_release();
    i2c_delay();
    if (!wait_scl_high())
        return false;
    i2c_delay();
    ack = !sda_read();
    scl_low();
    return ack;
}

static bool i2c_read_byte(uint8_t *value, bool acknowledge)
{
    unsigned int bit;
    uint8_t result = 0u;

    sda_release();
    for (bit = 0u; bit < 8u; ++bit)
    {
        result <<= 1;
        i2c_delay();
        if (!wait_scl_high())
            return false;
        if (sda_read())
            result |= 1u;
        i2c_delay();
        scl_low();
    }

    if (acknowledge)
        sda_low();
    else
        sda_release();

    i2c_delay();
    if (!wait_scl_high())
        return false;
    i2c_delay();
    scl_low();
    sda_release();
    *value = result;
    return true;
}

bool board_i2c_init(board_i2c_bus_t bus)
{
    if (bus != BOARD_I2C_BUS_IMU)
        return false;

    STM32_RCC->APB2ENR |= STM32_RCC_APB2_IOPBEN;
    (void)board_time_init();

    /* The PCB routes MPU_SDA to PB8 and MPU_SCL to PB9. This is intentionally
       implemented as software I2C: STM32F103 I2C1 remap expects PB8=SCL and
       PB9=SDA, the opposite signal assignment. External 4.7 kOhm pull-ups are
       present on the board. */
    stm32_gpio_write(STM32_GPIOB, I2C_SDA_PIN, 1);
    stm32_gpio_write(STM32_GPIOB, I2C_SCL_PIN, 1);
    stm32_gpio_config_nibble(STM32_GPIOB, I2C_SDA_PIN, 0x6u);
    stm32_gpio_config_nibble(STM32_GPIOB, I2C_SCL_PIN, 0x6u);
    board_delay_us(10u);

    i2c_initialized = sda_read() && scl_read();
    return i2c_initialized;
}

bool board_i2c_write_reg(board_i2c_bus_t bus,
                         uint8_t address_7bit,
                         uint8_t reg,
                         const uint8_t *data,
                         size_t length)
{
    size_t i;
    bool ok = false;

    if ((bus != BOARD_I2C_BUS_IMU) || (address_7bit > 0x7Fu) ||
        ((length != 0u) && (data == NULL)))
        return false;

    if (!i2c_initialized && !board_i2c_init(bus))
        return false;

    if (!i2c_start())
        goto done;
    if (!i2c_write_byte((uint8_t)(address_7bit << 1)))
        goto done;
    if (!i2c_write_byte(reg))
        goto done;

    for (i = 0u; i < length; ++i)
    {
        if (!i2c_write_byte(data[i]))
            goto done;
    }

    ok = true;
done:
    i2c_stop();
    return ok;
}

bool board_i2c_read_reg(board_i2c_bus_t bus,
                        uint8_t address_7bit,
                        uint8_t reg,
                        uint8_t *data,
                        size_t length)
{
    size_t i;
    bool ok = false;

    if ((bus != BOARD_I2C_BUS_IMU) || (address_7bit > 0x7Fu) ||
        (data == NULL) || (length == 0u))
        return false;

    if (!i2c_initialized && !board_i2c_init(bus))
        return false;

    if (!i2c_start())
        goto done;
    if (!i2c_write_byte((uint8_t)(address_7bit << 1)))
        goto done;
    if (!i2c_write_byte(reg))
        goto done;
    if (!i2c_start())
        goto done;
    if (!i2c_write_byte((uint8_t)((address_7bit << 1) | 1u)))
        goto done;

    for (i = 0u; i < length; ++i)
    {
        if (!i2c_read_byte(&data[i], i + 1u < length))
            goto done;
    }

    ok = true;
done:
    i2c_stop();
    return ok;
}
