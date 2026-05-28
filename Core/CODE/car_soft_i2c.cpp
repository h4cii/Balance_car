#include "car_soft_i2c.hpp"

#include "car_time.hpp"

namespace car {

SoftI2cBus soft_i2c;

void SoftI2cBus::init()
{
    GPIO_InitTypeDef gpio = {};

    micro_timer.init();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = IIC_SCL_Pin | IIC_SDA_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    sdaWrite(GPIO_PIN_SET);
    sclWrite(GPIO_PIN_SET);
}

bool SoftI2cBus::writeReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    if (!start()) {
        return false;
    }
    sendByte((uint8_t)(dev_addr << 1U));
    if (!waitAck()) {
        return false;
    }
    sendByte(reg_addr);
    if (!waitAck()) {
        return false;
    }
    sendByte(data);
    if (!waitAck()) {
        return false;
    }
    stop();
    return true;
}

bool SoftI2cBus::readReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t &data)
{
    return readRegs(dev_addr, reg_addr, &data, 1U);
}

bool SoftI2cBus::readRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    if ((data == nullptr) || (len == 0U)) {
        return false;
    }

    if (!start()) {
        return false;
    }
    sendByte((uint8_t)(dev_addr << 1U));
    if (!waitAck()) {
        return false;
    }
    sendByte(reg_addr);
    if (!waitAck()) {
        return false;
    }

    if (!start()) {
        return false;
    }
    sendByte((uint8_t)((dev_addr << 1U) | 1U));
    if (!waitAck()) {
        return false;
    }

    for (uint8_t i = 0U; i < len; ++i) {
        data[i] = readByte(i + 1U < len);
    }
    stop();
    return true;
}

void SoftI2cBus::delay()
{
    micro_timer.delayUs(I2C_DELAY_US);
}

void SoftI2cBus::sclWrite(GPIO_PinState state)
{
    HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, state);
}

void SoftI2cBus::sdaWrite(GPIO_PinState state)
{
    HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, state);
}

GPIO_PinState SoftI2cBus::sdaRead()
{
    return HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port, IIC_SDA_Pin);
}

bool SoftI2cBus::start()
{
    sdaWrite(GPIO_PIN_SET);
    sclWrite(GPIO_PIN_SET);
    delay();
    if (sdaRead() == GPIO_PIN_RESET) {
        return false;
    }
    sdaWrite(GPIO_PIN_RESET);
    delay();
    sclWrite(GPIO_PIN_RESET);
    return true;
}

void SoftI2cBus::stop()
{
    sdaWrite(GPIO_PIN_RESET);
    sclWrite(GPIO_PIN_RESET);
    delay();
    sclWrite(GPIO_PIN_SET);
    sdaWrite(GPIO_PIN_SET);
    delay();
}

void SoftI2cBus::ack()
{
    sclWrite(GPIO_PIN_RESET);
    sdaWrite(GPIO_PIN_RESET);
    delay();
    sclWrite(GPIO_PIN_SET);
    delay();
    sclWrite(GPIO_PIN_RESET);
    sdaWrite(GPIO_PIN_SET);
}

void SoftI2cBus::nack()
{
    sclWrite(GPIO_PIN_RESET);
    sdaWrite(GPIO_PIN_SET);
    delay();
    sclWrite(GPIO_PIN_SET);
    delay();
    sclWrite(GPIO_PIN_RESET);
}

bool SoftI2cBus::waitAck()
{
    uint8_t timeout = 0U;

    sdaWrite(GPIO_PIN_SET);
    delay();
    sclWrite(GPIO_PIN_SET);
    delay();
    while (sdaRead() == GPIO_PIN_SET) {
        if (++timeout > 50U) {
            stop();
            return false;
        }
        delay();
    }
    sclWrite(GPIO_PIN_RESET);
    return true;
}

void SoftI2cBus::sendByte(uint8_t data)
{
    for (uint8_t i = 0U; i < 8U; ++i) {
        sclWrite(GPIO_PIN_RESET);
        sdaWrite((data & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1U;
        delay();
        sclWrite(GPIO_PIN_SET);
        delay();
    }
    sclWrite(GPIO_PIN_RESET);
    sdaWrite(GPIO_PIN_SET);
}

uint8_t SoftI2cBus::readByte(bool ack_required)
{
    uint8_t data = 0U;

    sdaWrite(GPIO_PIN_SET);
    for (uint8_t i = 0U; i < 8U; ++i) {
        sclWrite(GPIO_PIN_RESET);
        delay();
        sclWrite(GPIO_PIN_SET);
        data <<= 1U;
        if (sdaRead() == GPIO_PIN_SET) {
            data |= 1U;
        }
        delay();
    }
    if (ack_required) {
        ack();
    } else {
        nack();
    }
    return data;
}

}  // namespace car
