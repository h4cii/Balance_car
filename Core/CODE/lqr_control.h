#ifndef LQR_CONTROL_H
#define LQR_CONTROL_H

#include <stdint.h>

typedef struct {
    float x_pose_m;
    float x_speed_mps;
    float angle_rad;
    float angle_rate_rads;
    float yaw_rad;
    float yaw_rate_rads;
    float left_accel;
    float right_accel;
    int16_t left_pwm;
    int16_t right_pwm;
} lqr_state_t;

void lqr_control_init(void);
void lqr_control_reset_pose(void);
void lqr_control_set_target(float speed_mps, float yaw_rate_rads);
lqr_state_t lqr_control_update(int16_t left_counts, int16_t right_counts, float pitch_rad);
lqr_state_t lqr_control_get_state(void);

#endif
