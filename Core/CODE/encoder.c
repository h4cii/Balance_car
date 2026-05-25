#include "encoder.h"

#include "balance_config.h"
#include "tim.h"

void balance_encoder_init(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

balance_encoder_counts_t balance_encoder_read(void)
{
    balance_encoder_counts_t counts;
    const int16_t left_raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    const int16_t right_raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);

    counts.left_counts = (int16_t)(left_raw * BALANCE_LEFT_ENCODER_SIGN);
    counts.right_counts = (int16_t)(right_raw * BALANCE_RIGHT_ENCODER_SIGN);
    return counts;
}
