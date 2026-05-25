#ifndef BALANCE_MOTOR_H
#define BALANCE_MOTOR_H

#include <stdint.h>

void balance_motor_init(void);
void balance_motor_stop(void);
void balance_motor_set(int16_t left_pwm, int16_t right_pwm);
int16_t balance_pwm_limit(int32_t value);

#endif
