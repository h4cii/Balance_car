#include "motor.h"

#include "balance_config.h"
#include "main.h"
#include "tim.h"

static int16_t abs16(int16_t value)
{
    return value < 0 ? (int16_t)-value : value;
}

static void write_dir(GPIO_TypeDef *port_high, uint16_t pin_high,
                      GPIO_TypeDef *port_low, uint16_t pin_low,
                      int16_t pwm)
{
    if (pwm > 0) {
        HAL_GPIO_WritePin(port_high, pin_high, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port_low, pin_low, GPIO_PIN_RESET);
    } else if (pwm < 0) {
        HAL_GPIO_WritePin(port_high, pin_high, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port_low, pin_low, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(port_high, pin_high, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port_low, pin_low, GPIO_PIN_RESET);
    }
}

int16_t balance_pwm_limit(int32_t value)
{
    if (value > BALANCE_PWM_MAX) {
        return BALANCE_PWM_MAX;
    }
    if (value < -BALANCE_PWM_MAX) {
        return -BALANCE_PWM_MAX;
    }
    return (int16_t)value;
}

void balance_motor_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    balance_motor_stop();
}

void balance_motor_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);

    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

void balance_motor_set(int16_t left_pwm, int16_t right_pwm)
{
    left_pwm = balance_pwm_limit((int32_t)left_pwm * BALANCE_LEFT_MOTOR_SIGN);
    right_pwm = balance_pwm_limit((int32_t)right_pwm * BALANCE_RIGHT_MOTOR_SIGN);

    write_dir(AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, left_pwm);
    write_dir(BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, right_pwm);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)abs16(left_pwm));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)abs16(right_pwm));
}
