#ifndef BALANCE_ENCODER_H
#define BALANCE_ENCODER_H

#include <stdint.h>

typedef struct {
    int16_t left_counts;
    int16_t right_counts;
} balance_encoder_counts_t;

void balance_encoder_init(void);
balance_encoder_counts_t balance_encoder_read(void);

#endif
