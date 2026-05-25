#ifndef CAR_REMOTE_CONTROL_HPP
#define CAR_REMOTE_CONTROL_HPP

#include <stdint.h>

#include "usart.h"

namespace car {

class RemoteControl final {
public:
    void init();
    void onByte(uint8_t byte);
    void onRxComplete(UART_HandleTypeDef *huart);

private:
    uint8_t rx_byte_ = 0U;
    bool fast_mode_ = false;
};

extern RemoteControl remote;

}  // namespace car

#endif
