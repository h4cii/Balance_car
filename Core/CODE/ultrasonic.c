#include "ultrasonic.h"

#include "balance_time.h"
#include "main.h"
#include "tim.h"

static volatile uint8_t waiting_for_falling;
static volatile uint16_t distance_mm;

void ultrasonic_init(void)
{
    waiting_for_falling = 0U;
    distance_mm = 0U;
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
}

void ultrasonic_trigger(void)
{
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
    balance_delay_us(2U);
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
    balance_delay_us(12U);
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
}

uint16_t ultrasonic_get_distance_mm(void)
{
    return distance_mm;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM3) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)) {
        if (!waiting_for_falling) {
            __HAL_TIM_SET_COUNTER(htim, 0U);
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
            waiting_for_falling = 1U;
        } else {
            const uint32_t pulse_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            uint32_t mm = (pulse_us * 343U) / 2000U;
            if (mm > 65535U) {
                mm = 65535U;
            }
            distance_mm = (uint16_t)mm;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
            waiting_for_falling = 0U;
        }
    }
}
