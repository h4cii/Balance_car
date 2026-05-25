#include "car_ultrasonic.hpp"

#include "car_time.hpp"
#include "main.h"

namespace car {

UltrasonicSensor ultrasonic;

void UltrasonicSensor::init()
{
    waiting_for_falling_ = 0U;
    distance_mm_ = 0U;
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
}

void UltrasonicSensor::trigger() const
{
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
    micro_timer.delayUs(2U);
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
    micro_timer.delayUs(12U);
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
}

uint16_t UltrasonicSensor::distanceMm() const
{
    return distance_mm_;
}

void UltrasonicSensor::onCapture(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance != TIM3) || (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_3)) {
        return;
    }

    if (!waiting_for_falling_) {
        __HAL_TIM_SET_COUNTER(htim, 0U);
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
        waiting_for_falling_ = 1U;
    } else {
        const uint32_t pulse_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        uint32_t mm = (pulse_us * 343U) / 2000U;
        if (mm > 65535U) {
            mm = 65535U;
        }
        distance_mm_ = (uint16_t)mm;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
        waiting_for_falling_ = 0U;
    }
}

}  // namespace car

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    car::ultrasonic.onCapture(htim);
}
