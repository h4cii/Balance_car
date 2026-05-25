#include "remote_control.h"

#include "balance_config.h"
#include "balance_control.h"
#include "usart.h"

static uint8_t rx_byte;
static uint8_t fast_mode;

void remote_control_init(void)
{
    fast_mode = 0U;
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
}

void remote_control_on_byte(uint8_t byte)
{
    const float speed = fast_mode ? BALANCE_REMOTE_SPEED_FAST_MPS : BALANCE_REMOTE_SPEED_SLOW_MPS;
    const float yaw = fast_mode ? BALANCE_REMOTE_YAW_FAST_RADS : BALANCE_REMOTE_YAW_SLOW_RADS;

    if (byte == 0x58U) {
        fast_mode = 1U;
        return;
    }
    if (byte == 0x59U) {
        fast_mode = 0U;
        return;
    }

    if ((byte == '0') || (byte == 0x30U)) {
        balance_control_set_mode(BALANCE_MODE_NORMAL);
        return;
    }
    if ((byte == '1') || (byte == 0x31U)) {
        balance_control_set_mode(BALANCE_MODE_ULTRASONIC_AVOID);
        return;
    }
    if ((byte == '2') || (byte == 0x32U)) {
        balance_control_set_mode(BALANCE_MODE_ULTRASONIC_FOLLOW);
        return;
    }

    if ((byte == 0x5AU) || (byte == 0x00U)) {
        balance_control_set_remote_target(0.0f, 0.0f, byte);
    } else if ((byte == 0x41U) || (byte == 0x01U)) {
        balance_control_set_remote_target(speed, 0.0f, byte);
    } else if ((byte == 0x45U) || (byte == 0x05U)) {
        balance_control_set_remote_target(-speed, 0.0f, byte);
    } else if ((byte == 0x42U) || (byte == 0x43U) || (byte == 0x44U) ||
               (byte == 0x02U) || (byte == 0x03U) || (byte == 0x04U)) {
        balance_control_set_remote_target(0.0f, yaw, byte);
    } else if ((byte == 0x46U) || (byte == 0x47U) || (byte == 0x48U) ||
               (byte == 0x06U) || (byte == 0x07U) || (byte == 0x08U)) {
        balance_control_set_remote_target(0.0f, -yaw, byte);
    } else {
        balance_control_set_remote_target(0.0f, 0.0f, byte);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        remote_control_on_byte(rx_byte);
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
    }
}
