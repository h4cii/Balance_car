#include "car_remote_control.hpp"

#include "car_config.hpp"
#include "car_control.hpp"

namespace car {

RemoteControl remote;

void RemoteControl::init()
{
    fast_mode_ = false;
    HAL_UART_Receive_IT(&huart3, &rx_byte_, 1U);
}

void RemoteControl::onByte(uint8_t byte)
{
    const float speed = fast_mode_ ? BALANCE_REMOTE_SPEED_FAST_MPS : BALANCE_REMOTE_SPEED_SLOW_MPS;
    const float yaw = (fast_mode_ ? BALANCE_REMOTE_YAW_FAST_RADS : BALANCE_REMOTE_YAW_SLOW_RADS) *
                      BALANCE_REMOTE_YAW_SIGN;

    if (isFastCommand(byte)) {
        fast_mode_ = true;
        return;
    }
    if (isSlowCommand(byte)) {
        fast_mode_ = false;
        return;
    }

    if (isNormalModeCommand(byte)) {
        controller.setMode(BalanceMode::normal);
        return;
    }
    if (isAvoidModeCommand(byte)) {
        controller.setMode(BalanceMode::ultrasonicAvoid);
        return;
    }
    if (isFollowModeCommand(byte)) {
        controller.setMode(BalanceMode::ultrasonicFollow);
        return;
    }
    if (isCycleModeCommand(byte)) {
        controller.cycleMode();
        return;
    }

    if (isStopCommand(byte)) {
        controller.setRemoteTarget(0.0f, 0.0f, byte);
    } else if (isForwardCommand(byte)) {
        controller.setRemoteTarget(speed, 0.0f, byte);
    } else if (isBackwardCommand(byte)) {
        controller.setRemoteTarget(-speed, 0.0f, byte);
    } else if (isRightCommand(byte)) {
        controller.setRemoteTarget(0.0f, yaw, byte);
    } else if (isLeftCommand(byte)) {
        controller.setRemoteTarget(0.0f, -yaw, byte);
    } else {
        controller.setRemoteTarget(0.0f, 0.0f, byte);
    }
}

bool RemoteControl::isFastCommand(uint8_t byte)
{
    return (byte == 'X') || (byte == 'x');
}

bool RemoteControl::isSlowCommand(uint8_t byte)
{
    return (byte == 'Y') || (byte == 'y');
}

bool RemoteControl::isStopCommand(uint8_t byte)
{
    return (byte == 'Z') || (byte == 'z') || (byte == 0x00U);
}

bool RemoteControl::isForwardCommand(uint8_t byte)
{
    return (byte == 'A') || (byte == 'a') || (byte == 0x01U);
}

bool RemoteControl::isBackwardCommand(uint8_t byte)
{
    return (byte == 'E') || (byte == 'e') || (byte == 0x05U);
}

bool RemoteControl::isRightCommand(uint8_t byte)
{
    return (byte == 'B') || (byte == 'C') || (byte == 'D') ||
           (byte == 'b') || (byte == 'c') || (byte == 'd') ||
           (byte == 0x02U) || (byte == 0x03U) || (byte == 0x04U);
}

bool RemoteControl::isLeftCommand(uint8_t byte)
{
    return (byte == 'F') || (byte == 'G') || (byte == 'H') ||
           (byte == 'f') || (byte == 'g') || (byte == 'h') ||
           (byte == 0x06U) || (byte == 0x07U) || (byte == 0x08U);
}

bool RemoteControl::isNormalModeCommand(uint8_t byte)
{
    return (byte == '0') || (byte == 'N') || (byte == 'n');
}

bool RemoteControl::isAvoidModeCommand(uint8_t byte)
{
    return (byte == '1') || (byte == 'O') || (byte == 'o');
}

bool RemoteControl::isFollowModeCommand(uint8_t byte)
{
    return (byte == '2') || (byte == 'P') || (byte == 'p');
}

bool RemoteControl::isCycleModeCommand(uint8_t byte)
{
    return (byte == 'M') || (byte == 'm');
}

void RemoteControl::onRxComplete(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        onByte(rx_byte_);
        HAL_UART_Receive_IT(&huart3, &rx_byte_, 1U);
    }
}

}  // namespace car

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    car::remote.onRxComplete(huart);
}
