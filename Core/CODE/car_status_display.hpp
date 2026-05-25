#ifndef CAR_STATUS_DISPLAY_HPP
#define CAR_STATUS_DISPLAY_HPP

#include <stdint.h>

#include "main.h"

namespace car {

class StatusDisplay final {
public:
    void init();
    void update();

private:
    static void scl(GPIO_PinState state);
    static void sda(GPIO_PinState state);
    static void dc(GPIO_PinState state);
    static void writeByte(uint8_t value);
    static void cmd(uint8_t command);
    static void data(uint8_t value);
    static void setPos(uint8_t page, uint8_t col);
    static void clear();
    static const uint8_t *glyphFor(char c);
    static void writeText(uint8_t page, const char *text);
    static void formatTenths(char *buf, unsigned int len, long value10);

    bool ready_ = false;
    uint32_t last_ms_ = 0U;
};

extern StatusDisplay status_display;

}  // namespace car

#endif
