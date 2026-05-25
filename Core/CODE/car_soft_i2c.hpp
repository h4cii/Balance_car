#ifndef CAR_SOFT_I2C_HPP
#define CAR_SOFT_I2C_HPP

#include <stdint.h>

#include "main.h"

namespace car {

class SoftI2cBus final {
public:
    void init();
    bool writeReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
    bool readReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t &data);
    bool readRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

private:
    static constexpr uint32_t I2C_DELAY_US = 2U;

    static void delay();
    static void sclWrite(GPIO_PinState state);
    static void sdaWrite(GPIO_PinState state);
    static GPIO_PinState sdaRead();
    static void stop();
    static void ack();
    static void nack();
    static void sendByte(uint8_t data);
    static uint8_t readByte(bool ack_required);

    bool start();
    bool waitAck();
};

extern SoftI2cBus soft_i2c;

}  // namespace car

#endif
