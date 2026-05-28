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
    static bool isFastCommand(uint8_t byte);
    static bool isSlowCommand(uint8_t byte);
    static bool isStopCommand(uint8_t byte);
    static bool isForwardCommand(uint8_t byte);
    static bool isBackwardCommand(uint8_t byte);
    static bool isRightCommand(uint8_t byte);
    static bool isLeftCommand(uint8_t byte);
    static bool isNormalModeCommand(uint8_t byte);
    static bool isAvoidModeCommand(uint8_t byte);
    static bool isFollowModeCommand(uint8_t byte);
    static bool isCycleModeCommand(uint8_t byte);

    uint8_t rx_byte_ = 0U;
    bool fast_mode_ = false;
};

extern RemoteControl remote;

}  // namespace car

#endif
