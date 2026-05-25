#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>

void remote_control_init(void);
void remote_control_on_byte(uint8_t byte);

#endif
