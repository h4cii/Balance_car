#ifndef CAR_MOTOR_HPP
#define CAR_MOTOR_HPP

#include <stdint.h>

#include "main.h"

namespace car {

class MotorDriver final {
public:
    void init();
    void stop();
    void set(int16_t left_pwm, int16_t right_pwm);

    static int16_t limit(int32_t value);

private:
    static int16_t abs16(int16_t value);
    static void writeDir(GPIO_TypeDef *port_high, uint16_t pin_high,
                         GPIO_TypeDef *port_low, uint16_t pin_low,
                         int16_t pwm);
};

extern MotorDriver motor;

}  // namespace car

#endif
