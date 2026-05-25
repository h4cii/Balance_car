#include "balance_time.h"

#include "main.h"

#define DWT_CTRL      (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CYCCNT    (*(volatile uint32_t *)0xE0001004UL)
#define DEM_CR        (*(volatile uint32_t *)0xE000EDFCUL)

static uint32_t cycles_per_us = 72U;

void balance_time_init(void)
{
    DEM_CR |= (1UL << 24);
    DWT_CYCCNT = 0U;
    DWT_CTRL |= 1UL;
    cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    if (cycles_per_us == 0U) {
        cycles_per_us = 1U;
    }
}

void balance_delay_us(uint32_t us)
{
    const uint32_t start = DWT_CYCCNT;
    const uint32_t ticks = us * cycles_per_us;
    while ((uint32_t)(DWT_CYCCNT - start) < ticks) {
    }
}
