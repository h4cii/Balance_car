#ifndef CAR_OLED_UI_HPP
#define CAR_OLED_UI_HPP

#include <stdint.h>

namespace car {

struct OledUiData {
    bool enabled = false;
    bool mpu_ok = false;
    uint16_t battery_centivolts = 0U;
    int16_t pitch_deg_x10 = 0;
    int16_t pwm_left = 0;
    int16_t pwm_right = 0;
    int16_t encoder_left = 0;
    int16_t encoder_right = 0;
    uint16_t ultrasonic_mm = 0U;
    bool ultrasonic_valid = false;
    uint8_t mode = 0U;
};

class OledUiDemo final {
public:
    void init();
    void update(const OledUiData &data);

private:
    void render(const OledUiData &data);

    uint16_t frame_ = 0U;
};

extern OledUiDemo oled_ui_demo;

}  // namespace car

#endif
