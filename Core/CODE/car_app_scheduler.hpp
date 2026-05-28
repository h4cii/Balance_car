#ifndef CAR_APP_SCHEDULER_HPP
#define CAR_APP_SCHEDULER_HPP

#include <stdint.h>

#include "tim.h"

namespace car {

class AppScheduler final {
public:
    void start();
    bool takeImuTick();
    bool takeOledTick();
    bool takeTelemetryTick();
    bool takeUltrasonicTick();
    void onTimerElapsed(TIM_HandleTypeDef *htim);

private:
    static bool take(volatile uint8_t &flag);

    volatile uint8_t imu_tick_ = 0U;
    volatile uint8_t oled_tick_ = 0U;
    volatile uint8_t telemetry_tick_ = 0U;
    volatile uint8_t ultrasonic_tick_ = 0U;
    volatile uint16_t tick_5ms_ = 0U;
};

extern AppScheduler app_scheduler;

}  // namespace car

#endif
