#include "car_app_scheduler.hpp"

namespace car {

AppScheduler app_scheduler;

void AppScheduler::start()
{
    __disable_irq();
    imu_tick_ = 0U;
    oled_tick_ = 0U;
    telemetry_tick_ = 0U;
    ultrasonic_tick_ = 0U;
    tick_5ms_ = 0U;
    __enable_irq();

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    HAL_TIM_Base_Start_IT(&htim2);
}

bool AppScheduler::take(volatile uint8_t &flag)
{
    __disable_irq();
    const bool ready = (flag != 0U);
    if (ready) {
        flag = 0U;
    }
    __enable_irq();
    return ready;
}

bool AppScheduler::takeImuTick()
{
    return take(imu_tick_);
}

bool AppScheduler::takeOledTick()
{
    return take(oled_tick_);
}

bool AppScheduler::takeTelemetryTick()
{
    return take(telemetry_tick_);
}

bool AppScheduler::takeUltrasonicTick()
{
    return take(ultrasonic_tick_);
}

void AppScheduler::onTimerElapsed(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) {
        return;
    }

    tick_5ms_++;
    imu_tick_ = 1U;

    if ((tick_5ms_ % 10U) == 0U) {
        oled_tick_ = 1U;
    }
    if ((tick_5ms_ % 20U) == 0U) {
        telemetry_tick_ = 1U;
        ultrasonic_tick_ = 1U;
    }

    if (tick_5ms_ >= 200U) {
        tick_5ms_ = 0U;
    }
}

}  // namespace car

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    car::app_scheduler.onTimerElapsed(htim);
}
