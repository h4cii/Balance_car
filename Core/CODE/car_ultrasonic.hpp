#ifndef CAR_ULTRASONIC_HPP
#define CAR_ULTRASONIC_HPP

#include <stdint.h>

#include "tim.h"

namespace car {

class UltrasonicSensor final {
public:
    void init();
    void trigger() const;
    uint16_t distanceMm() const;
    void onCapture(TIM_HandleTypeDef *htim);

private:
    volatile uint8_t waiting_for_falling_ = 0U;
    volatile uint16_t distance_mm_ = 0U;
};

extern UltrasonicSensor ultrasonic;

}  // namespace car

#endif
