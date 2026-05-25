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
    const float yaw = fast_mode_ ? BALANCE_REMOTE_YAW_FAST_RADS : BALANCE_REMOTE_YAW_SLOW_RADS;

    if (byte == 0x58U) {
        fast_mode_ = true;
        return;
    }
    if (byte == 0x59U) {
        fast_mode_ = false;
        return;
    }

    if ((byte == '0') || (byte == 0x30U)) {
        controller.setMode(BalanceMode::normal);
        return;
    }
    if ((byte == '1') || (byte == 0x31U)) {
        controller.setMode(BalanceMode::ultrasonicAvoid);
        return;
    }
    if ((byte == '2') || (byte == 0x32U)) {
        controller.setMode(BalanceMode::ultrasonicFollow);
        return;
    }

    if ((byte == 0x5AU) || (byte == 0x00U)) {
        controller.setRemoteTarget(0.0f, 0.0f, byte);
    } else if ((byte == 0x41U) || (byte == 0x01U)) {
        controller.setRemoteTarget(speed, 0.0f, byte);
    } else if ((byte == 0x45U) || (byte == 0x05U)) {
        controller.setRemoteTarget(-speed, 0.0f, byte);
    } else if ((byte == 0x42U) || (byte == 0x43U) || (byte == 0x44U) ||
               (byte == 0x02U) || (byte == 0x03U) || (byte == 0x04U)) {
        controller.setRemoteTarget(0.0f, yaw, byte);
    } else if ((byte == 0x46U) || (byte == 0x47U) || (byte == 0x48U) ||
               (byte == 0x06U) || (byte == 0x07U) || (byte == 0x08U)) {
        controller.setRemoteTarget(0.0f, -yaw, byte);
    } else {
        controller.setRemoteTarget(0.0f, 0.0f, byte);
    }
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
