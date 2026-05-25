#ifndef BALANCE_BATTERY_H
#define BALANCE_BATTERY_H

#include <stdint.h>

void balance_battery_init(void);
uint16_t balance_battery_read_centivolts(void);

#endif
