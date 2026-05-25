#include "car_motor.hpp"

#include "car_config.hpp"
#include "tim.h"

namespace car {

MotorDriver motor;

void MotorDriver::init()
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    stop();
}

void MotorDriver::stop()
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);

    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
}

void MotorDriver::set(int16_t left_pwm, int16_t right_pwm)
{
    left_pwm = limit((int32_t)left_pwm * BALANCE_LEFT_MOTOR_SIGN);
    right_pwm = limit((int32_t)right_pwm * BALANCE_RIGHT_MOTOR_SIGN);

    writeDir(AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin, left_pwm);
    writeDir(BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin, right_pwm);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)abs16(left_pwm));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint32_t)abs16(right_pwm));
}

int16_t MotorDriver::limit(int32_t value)
{
    if (value > BALANCE_PWM_MAX) {
        return BALANCE_PWM_MAX;
    }
    if (value < -BALANCE_PWM_MAX) {
        return -BALANCE_PWM_MAX;
    }
    return (int16_t)value;
}

int16_t MotorDriver::abs16(int16_t value)
{
    return value < 0 ? (int16_t)-value : value;
}

void MotorDriver::writeDir(GPIO_TypeDef *port_high, uint16_t pin_high,
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

}  // namespace car
