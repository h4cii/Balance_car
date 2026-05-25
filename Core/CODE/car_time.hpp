#ifndef CAR_TIME_HPP
#define CAR_TIME_HPP

#include <stdint.h>

namespace car {

class MicrosecondTimer final {
public:
    void init();
    void delayUs(uint32_t us) const;

private:
    static constexpr uint32_t DWT_CTRL = 0xE0001000UL;
    static constexpr uint32_t DWT_CYCCNT = 0xE0001004UL;
    static constexpr uint32_t DEM_CR = 0xE000EDFCUL;

    static volatile uint32_t &reg(uint32_t address);

    uint32_t cycles_per_us_ = 72U;
};

extern MicrosecondTimer micro_timer;

}  // namespace car

#endif
