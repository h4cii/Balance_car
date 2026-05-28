#include "car_ultrasonic.hpp"

#include "car_config.hpp"
#include "car_time.hpp"
#include "main.h"

namespace car {

UltrasonicSensor ultrasonic;

namespace {

void writeTriggerPins(GPIO_PinState state)
{
    HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, state);
    HAL_GPIO_WritePin(TriggerAlt_GPIO_Port, TriggerAlt_Pin, state);
}

GPIO_PinState readEchoPin(uint16_t gpio_pin)
{
    if (gpio_pin == UltrasonicCapture_Pin) {
        return HAL_GPIO_ReadPin(UltrasonicCapture_GPIO_Port, UltrasonicCapture_Pin);
    }
    if (gpio_pin == UltrasonicCaptureAlt_Pin) {
        return HAL_GPIO_ReadPin(UltrasonicCaptureAlt_GPIO_Port, UltrasonicCaptureAlt_Pin);
    }
    return GPIO_PIN_RESET;
}

}  // namespace

void UltrasonicSensor::init()
{
    GPIO_InitTypeDef gpio = {};

    micro_timer.init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    writeTriggerPins(GPIO_PIN_RESET);

    gpio.Pin = Trigger_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(Trigger_GPIO_Port, &gpio);

    gpio.Pin = TriggerAlt_Pin;
    HAL_GPIO_Init(TriggerAlt_GPIO_Port, &gpio);

    gpio.Pin = UltrasonicCapture_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(UltrasonicCapture_GPIO_Port, &gpio);

    gpio.Pin = UltrasonicCaptureAlt_Pin;
    HAL_GPIO_Init(UltrasonicCaptureAlt_GPIO_Port, &gpio);

    clearMeasurement();
    __HAL_GPIO_EXTI_CLEAR_IT(UltrasonicCapture_Pin);
    __HAL_GPIO_EXTI_CLEAR_IT(UltrasonicCaptureAlt_Pin);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(UltrasonicCaptureAlt_EXTI_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(UltrasonicCaptureAlt_EXTI_IRQn);
}

void UltrasonicSensor::trigger()
{
    const uint32_t now = HAL_GetTick();
    if (measurement_active_ &&
        ((uint32_t)(now - trigger_ms_) < BALANCE_ULTRASONIC_ECHO_TIMEOUT_MS)) {
        return;
    }

    if (measurement_active_) {
        clearMeasurement();
    }

    measurement_active_ = 1U;
    waiting_for_falling_ = 0U;
    trigger_ms_ = now;

    writeTriggerPins(GPIO_PIN_RESET);
    micro_timer.delayUs(2U);
    writeTriggerPins(GPIO_PIN_SET);
    micro_timer.delayUs(15U);
    writeTriggerPins(GPIO_PIN_RESET);
}

uint16_t UltrasonicSensor::distanceMm() const
{
    if (!has_distance_) {
        return 0U;
    }
    return distance_mm_;
}

bool UltrasonicSensor::hasFreshDistance(uint32_t now_ms) const
{
    return has_distance_ &&
           ((uint32_t)(now_ms - last_echo_ms_) <= BALANCE_ULTRASONIC_VALID_MS);
}

void UltrasonicSensor::onEchoEdge(uint16_t gpio_pin)
{
    if (!measurement_active_) {
        return;
    }

    if (readEchoPin(gpio_pin) == GPIO_PIN_SET) {
        if (waiting_for_falling_ && (active_echo_pin_ != gpio_pin)) {
            return;
        }
        rising_ticks_ = micro_timer.ticks();
        waiting_for_falling_ = 1U;
        active_echo_pin_ = gpio_pin;
        return;
    }

    if (!waiting_for_falling_ || (active_echo_pin_ != gpio_pin)) {
        return;
    }

    const uint32_t pulse_ticks = micro_timer.ticks() - rising_ticks_;
    const uint32_t pulse_us = micro_timer.ticksToUs(pulse_ticks);
    uint32_t mm = (pulse_us * 343U) / 2000U;
    if (mm > BALANCE_ULTRASONIC_MAX_MM) {
        mm = BALANCE_ULTRASONIC_MAX_MM;
    }

    distance_mm_ = (uint16_t)mm;
    last_echo_ms_ = HAL_GetTick();
    has_distance_ = 1U;
    measurement_active_ = 0U;
    waiting_for_falling_ = 0U;
    active_echo_pin_ = 0U;
}

void UltrasonicSensor::clearMeasurement()
{
    measurement_active_ = 0U;
    waiting_for_falling_ = 0U;
    has_distance_ = 0U;
    active_echo_pin_ = 0U;
    trigger_ms_ = 0U;
    rising_ticks_ = 0U;
    last_echo_ms_ = 0U;
    distance_mm_ = 0U;
}

}  // namespace car
