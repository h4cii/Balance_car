#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

void ultrasonic_init(void);
void ultrasonic_trigger(void);
uint16_t ultrasonic_get_distance_mm(void);

#endif
