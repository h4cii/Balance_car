#include "soft_i2c.h"

#include "balance_time.h"
#include "main.h"

#define I2C_DELAY_US 2U

static void i2c_delay(void)
{
    balance_delay_us(I2C_DELAY_US);
}

static void scl_write(GPIO_PinState state)
{
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, state);
}

static void sda_write(GPIO_PinState state)
{
    HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, state);
}

static GPIO_PinState sda_read(void)
{
    return HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port, IIC_SDA_Pin);
}

void soft_i2c_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = IIC_SCL_Pin | IIC_SDA_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    sda_write(GPIO_PIN_SET);
    scl_write(GPIO_PIN_SET);
}

static uint8_t i2c_start(void)
{
    sda_write(GPIO_PIN_SET);
    scl_write(GPIO_PIN_SET);
    i2c_delay();
    if (sda_read() == GPIO_PIN_RESET) {
        return 0U;
    }
    sda_write(GPIO_PIN_RESET);
    i2c_delay();
    scl_write(GPIO_PIN_RESET);
    return 1U;
}

static void i2c_stop(void)
{
    sda_write(GPIO_PIN_RESET);
    scl_write(GPIO_PIN_RESET);
    i2c_delay();
    scl_write(GPIO_PIN_SET);
    sda_write(GPIO_PIN_SET);
    i2c_delay();
}

static void i2c_ack(void)
{
    scl_write(GPIO_PIN_RESET);
    sda_write(GPIO_PIN_RESET);
    i2c_delay();
    scl_write(GPIO_PIN_SET);
    i2c_delay();
    scl_write(GPIO_PIN_RESET);
    sda_write(GPIO_PIN_SET);
}

static void i2c_nack(void)
{
    scl_write(GPIO_PIN_RESET);
    sda_write(GPIO_PIN_SET);
    i2c_delay();
    scl_write(GPIO_PIN_SET);
    i2c_delay();
    scl_write(GPIO_PIN_RESET);
}

static uint8_t i2c_wait_ack(void)
{
    uint8_t timeout = 0U;

    sda_write(GPIO_PIN_SET);
    i2c_delay();
    scl_write(GPIO_PIN_SET);
    i2c_delay();
    while (sda_read() == GPIO_PIN_SET) {
        if (++timeout > 50U) {
            i2c_stop();
            return 0U;
        }
        i2c_delay();
    }
    scl_write(GPIO_PIN_RESET);
    return 1U;
}

static void i2c_send_byte(uint8_t data)
{
    for (uint8_t i = 0U; i < 8U; ++i) {
        scl_write(GPIO_PIN_RESET);
        sda_write((data & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1U;
        i2c_delay();
        scl_write(GPIO_PIN_SET);
        i2c_delay();
    }
    scl_write(GPIO_PIN_RESET);
    sda_write(GPIO_PIN_SET);
}

static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t data = 0U;

    sda_write(GPIO_PIN_SET);
    for (uint8_t i = 0U; i < 8U; ++i) {
        scl_write(GPIO_PIN_RESET);
        i2c_delay();
        scl_write(GPIO_PIN_SET);
        data <<= 1U;
        if (sda_read() == GPIO_PIN_SET) {
            data |= 1U;
        }
        i2c_delay();
    }
    if (ack) {
        i2c_ack();
    } else {
        i2c_nack();
    }
    return data;
}

uint8_t soft_i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    if (!i2c_start()) {
        return 0U;
    }
    i2c_send_byte((uint8_t)(dev_addr << 1U));
    if (!i2c_wait_ack()) {
        return 0U;
    }
    i2c_send_byte(reg_addr);
    if (!i2c_wait_ack()) {
        return 0U;
    }
    i2c_send_byte(data);
    if (!i2c_wait_ack()) {
        return 0U;
    }
    i2c_stop();
    return 1U;
}

uint8_t soft_i2c_read_regs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    if ((data == 0) || (len == 0U)) {
        return 0U;
    }

    if (!i2c_start()) {
        return 0U;
    }
    i2c_send_byte((uint8_t)(dev_addr << 1U));
    if (!i2c_wait_ack()) {
        return 0U;
    }
    i2c_send_byte(reg_addr);
    if (!i2c_wait_ack()) {
        return 0U;
    }

    if (!i2c_start()) {
        return 0U;
    }
    i2c_send_byte((uint8_t)((dev_addr << 1U) | 1U));
    if (!i2c_wait_ack()) {
        return 0U;
    }

    for (uint8_t i = 0U; i < len; ++i) {
        data[i] = i2c_read_byte((uint8_t)(i + 1U < len));
    }
    i2c_stop();
    return 1U;
}

uint8_t soft_i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    return soft_i2c_read_regs(dev_addr, reg_addr, data, 1U);
}
