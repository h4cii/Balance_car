#ifndef CAR_ULTRASONIC_HPP
#define CAR_ULTRASONIC_HPP

#include <stdint.h>

namespace car {

class UltrasonicSensor final {
public:
    void init();
    void trigger();
    uint16_t distanceMm() const;
    bool hasFreshDistance(uint32_t now_ms) const;
    void onEchoEdge(uint16_t gpio_pin);

private:
    void clearMeasurement();

    volatile uint8_t measurement_active_ = 0U;
    volatile uint8_t waiting_for_falling_ = 0U;
    volatile uint8_t has_distance_ = 0U;
    volatile uint16_t active_echo_pin_ = 0U;
    volatile uint32_t trigger_ms_ = 0U;
    volatile uint32_t rising_ticks_ = 0U;
    volatile uint32_t last_echo_ms_ = 0U;
    volatile uint16_t distance_mm_ = 0U;
};

extern UltrasonicSensor ultrasonic;

}  // namespace car

#endif
