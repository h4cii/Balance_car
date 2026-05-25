#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include <stdint.h>

typedef enum {
    BALANCE_MODE_NORMAL = 0,
    BALANCE_MODE_ULTRASONIC_AVOID = 1,
    BALANCE_MODE_ULTRASONIC_FOLLOW = 2
} balance_mode_t;

typedef enum {
    BALANCE_STOP_USER = 0,
    BALANCE_STOP_TILT = 1,
    BALANCE_STOP_LOW_BATTERY = 2,
    BALANCE_STOP_MPU_ERROR = 3
} balance_stop_reason_t;

typedef struct {
    uint8_t initialized;
    uint8_t enabled;
    uint8_t mpu_ok;
    uint8_t mpu_whoami;
    balance_mode_t mode;
    balance_stop_reason_t stop_reason;
    uint32_t control_ticks;
    float pitch_rad;
    float pitch_deg;
    float angle_rate_rads;
    float x_pose_m;
    float x_speed_mps;
    float yaw_rad;
    float yaw_rate_rads;
    uint16_t ultrasonic_mm;
    uint16_t battery_centivolts;
    int16_t encoder_left;
    int16_t encoder_right;
    int16_t pwm_left;
    int16_t pwm_right;
    uint8_t remote_last_byte;
} balance_state_t;

void balance_control_init(void);
void balance_control_poll(void);
void balance_control_handle_imu_interrupt(void);
void balance_control_set_enabled(uint8_t enabled);
void balance_control_toggle_enabled(void);
void balance_control_set_mode(balance_mode_t mode);
void balance_control_set_remote_target(float speed_mps, float yaw_rate_rads, uint8_t last_byte);
void balance_control_get_state(balance_state_t *out_state);

#endif
