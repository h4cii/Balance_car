#include "car_encoder.hpp"

#include "car_config.hpp"
#include "tim.h"

namespace car {

EncoderReader encoder;

void EncoderReader::init()
{
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

EncoderCounts EncoderReader::read()
{
    const int16_t left_raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    const int16_t right_raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);

    return {
        (int16_t)(left_raw * BALANCE_LEFT_ENCODER_SIGN),
        (int16_t)(right_raw * BALANCE_RIGHT_ENCODER_SIGN),
    };
}

}  // namespace car
