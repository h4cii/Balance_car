#include "lqr_control.h"

#include "balance_config.h"
#include "motor.h"

static lqr_state_t lqr_state;
static float target_speed_mps;
static float target_yaw_rate_rads;
static float last_pitch_rad;

static int16_t pwm_from_float(float value)
{
    if (value >= 0.0f) {
        return balance_pwm_limit((int32_t)(value + 0.5f));
    }
    return balance_pwm_limit((int32_t)(value - 0.5f));
}

void lqr_control_init(void)
{
    lqr_state.x_pose_m = 0.0f;
    lqr_state.x_speed_mps = 0.0f;
    lqr_state.angle_rad = 0.0f;
    lqr_state.angle_rate_rads = 0.0f;
    lqr_state.yaw_rad = 0.0f;
    lqr_state.yaw_rate_rads = 0.0f;
    lqr_state.left_accel = 0.0f;
    lqr_state.right_accel = 0.0f;
    lqr_state.left_pwm = 0;
    lqr_state.right_pwm = 0;
    target_speed_mps = 0.0f;
    target_yaw_rate_rads = 0.0f;
    last_pitch_rad = 0.0f;
}

void lqr_control_reset_pose(void)
{
    lqr_state.x_pose_m = 0.0f;
    lqr_state.yaw_rad = 0.0f;
}

void lqr_control_set_target(float speed_mps, float yaw_rate_rads)
{
    if ((speed_mps != target_speed_mps) || (yaw_rate_rads != target_yaw_rate_rads)) {
        lqr_control_reset_pose();
    }
    target_speed_mps = speed_mps;
    target_yaw_rate_rads = yaw_rate_rads;
}

lqr_state_t lqr_control_update(int16_t left_counts, int16_t right_counts, float pitch_rad)
{
    const float wheel_circ_m = BALANCE_PI * BALANCE_WHEEL_DIAMETER_MM / 1000.0f;
    const float track_m = BALANCE_WHEEL_SPACING_MM / 1000.0f;
    const float average_counts = ((float)left_counts + (float)right_counts) * 0.5f;
    const float diff_counts = (float)right_counts - (float)left_counts;

    lqr_state.x_speed_mps = average_counts * wheel_circ_m * BALANCE_CONTROL_FREQUENCY_HZ /
                            BALANCE_ENCODER_COUNTS_PER_REV;
    lqr_state.x_pose_m += lqr_state.x_speed_mps * BALANCE_SAMPLE_PERIOD_S;

    lqr_state.angle_rad = pitch_rad;
    lqr_state.angle_rate_rads = (pitch_rad - last_pitch_rad) * BALANCE_CONTROL_FREQUENCY_HZ;
    last_pitch_rad = pitch_rad;

    lqr_state.yaw_rate_rads = diff_counts * wheel_circ_m * BALANCE_CONTROL_FREQUENCY_HZ /
                              (BALANCE_ENCODER_COUNTS_PER_REV * track_m);
    lqr_state.yaw_rad += lqr_state.yaw_rate_rads * BALANCE_SAMPLE_PERIOD_S;

    lqr_state.left_accel = -(BALANCE_LQR_K1 * lqr_state.x_pose_m +
                             BALANCE_LQR_K2 * (lqr_state.x_speed_mps - target_speed_mps) +
                             BALANCE_LQR_K3 * (lqr_state.angle_rad - BALANCE_TARGET_ANGLE_RAD) +
                             BALANCE_LQR_K4 * lqr_state.angle_rate_rads +
                             BALANCE_LQR_K5 * lqr_state.yaw_rad +
                             BALANCE_LQR_K6 * (lqr_state.yaw_rate_rads - target_yaw_rate_rads));

    lqr_state.right_accel = -(BALANCE_LQR_K1 * lqr_state.x_pose_m +
                              BALANCE_LQR_K2 * (lqr_state.x_speed_mps - target_speed_mps) +
                              BALANCE_LQR_K3 * (lqr_state.angle_rad - BALANCE_TARGET_ANGLE_RAD) +
                              BALANCE_LQR_K4 * lqr_state.angle_rate_rads -
                              BALANCE_LQR_K5 * lqr_state.yaw_rad -
                              BALANCE_LQR_K6 * (lqr_state.yaw_rate_rads - target_yaw_rate_rads));

    lqr_state.left_pwm = pwm_from_float(BALANCE_RATIO_ACCEL *
                                        (lqr_state.x_speed_mps + lqr_state.left_accel * BALANCE_SAMPLE_PERIOD_S));
    lqr_state.right_pwm = pwm_from_float(BALANCE_RATIO_ACCEL *
                                         (lqr_state.x_speed_mps + lqr_state.right_accel * BALANCE_SAMPLE_PERIOD_S));
    return lqr_state;
}

lqr_state_t lqr_control_get_state(void)
{
    return lqr_state;
}
