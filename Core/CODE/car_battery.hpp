#ifndef CAR_BATTERY_HPP
#define CAR_BATTERY_HPP

#include <stdint.h>

namespace car {

class BatteryMonitor final {
public:
    void init();
    uint16_t readCentivolts();
};

extern BatteryMonitor battery;

}  // namespace car

#endif
