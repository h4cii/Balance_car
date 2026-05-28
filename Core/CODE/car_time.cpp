#include "car_time.hpp"

#include "main.h"

namespace car {

MicrosecondTimer micro_timer;

volatile uint32_t &MicrosecondTimer::reg(uint32_t address)
{
    return *reinterpret_cast<volatile uint32_t *>(address);
}

void MicrosecondTimer::init()
{
    reg(DEM_CR) |= (1UL << 24);
    reg(DWT_CYCCNT) = 0U;
    reg(DWT_CTRL) |= 1UL;
    cycles_per_us_ = HAL_RCC_GetHCLKFreq() / 1000000U;
    if (cycles_per_us_ == 0U) {
        cycles_per_us_ = 1U;
    }
}

void MicrosecondTimer::delayUs(uint32_t us) const
{
    const uint32_t start = reg(DWT_CYCCNT);
    const uint32_t ticks = us * cycles_per_us_;
    while ((uint32_t)(reg(DWT_CYCCNT) - start) < ticks) {
    }
}

uint32_t MicrosecondTimer::ticks() const
{
    return reg(DWT_CYCCNT);
}

uint32_t MicrosecondTimer::ticksToUs(uint32_t ticks) const
{
    if (cycles_per_us_ == 0U) {
        return ticks;
    }
    return ticks / cycles_per_us_;
}

}  // namespace car
