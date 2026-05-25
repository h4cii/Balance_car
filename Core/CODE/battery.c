#include "battery.h"

#include "adc.h"
#include "balance_config.h"

void balance_battery_init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1);
}

uint16_t balance_battery_read_centivolts(void)
{
    uint32_t raw;
    float centivolts;

    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return 0U;
    }
    raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    centivolts = ((float)raw * 3.3f * BALANCE_BATTERY_DIVIDER * 100.0f) / 4096.0f;
    if (centivolts < 0.0f) {
        return 0U;
    }
    if (centivolts > 65535.0f) {
        return 65535U;
    }
    return (uint16_t)(centivolts + 0.5f);
}
