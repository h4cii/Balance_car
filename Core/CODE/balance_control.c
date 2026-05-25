#include "balance_control.h"

#include <math.h>
#include <string.h>

#include "balance_config.h"
#include "balance_time.h"
#include "battery.h"
#include "encoder.h"
#include "kalman.h"
#include "lqr_control.h"
#include "main.h"
#include "motor.h"
#include "mpu6050_raw.h"
#include "remote_control.h"
#include "status_display.h"
#include "ultrasonic.h"

static balance_state_t state;
static float remote_speed_target_mps;
static float remote_yaw_target_rads;

static void stop_with_reason(balance_stop_reason_t reason)
{
    state.enabled = 0U;
    state.stop_reason = reason;
    remote_speed_target_mps = 0.0f;
    remote_yaw_target_rads = 0.0f;
    lqr_control_set_target(0.0f, 0.0f);
    lqr_control_reset_pose();
    balance_motor_stop();
}

static void apply_control_target(void)
{
    float speed = remote_speed_target_mps;
    float yaw_rate = remote_yaw_target_rads;

    if (state.mode == BALANCE_MODE_ULTRASONIC_AVOID) {
        if ((state.ultrasonic_mm > 0U) && (state.ultrasonic_mm < BALANCE_ULTRASONIC_AVOID_MM)) {
            speed = -BALANCE_REMOTE_SPEED_SLOW_MPS;
            yaw_rate = 0.0f;
        }
    } else if (state.mode == BALANCE_MODE_ULTRASONIC_FOLLOW) {
        if ((state.ultrasonic_mm > BALANCE_ULTRASONIC_FOLLOW_FAR_MM) &&
            (state.ultrasonic_mm < 1500U)) {
            speed = BALANCE_REMOTE_SPEED_SLOW_MPS;
        } else if ((state.ultrasonic_mm > 0U) &&
                   (state.ultrasonic_mm < BALANCE_ULTRASONIC_FOLLOW_NEAR_MM)) {
            speed = -BALANCE_REMOTE_SPEED_SLOW_MPS;
        } else {
            speed = 0.0f;
        }
        yaw_rate = 0.0f;
    }

    lqr_control_set_target(speed, yaw_rate);
}

void balance_control_init(void)
{
    memset(&state, 0, sizeof(state));
    state.stop_reason = BALANCE_STOP_USER;
    state.mode = BALANCE_MODE_NORMAL;

    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(MPU6050_INT_Pin);

    balance_time_init();
    balance_motor_init();
    balance_encoder_init();
    balance_battery_init();
    lqr_control_init();
    mpu6050_raw_bus_init();
    ultrasonic_init();
    remote_control_init();
    status_display_init();

    state.battery_centivolts = balance_battery_read_centivolts();
    state.mpu_whoami = mpu6050_raw_read_whoami();
    state.mpu_ok = mpu6050_raw_init();
    if (!state.mpu_ok) {
        stop_with_reason(BALANCE_STOP_MPU_ERROR);
    }

    state.initialized = 1U;
    if (state.mpu_ok) {
        __HAL_GPIO_EXTI_CLEAR_IT(MPU6050_INT_Pin);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
}

void balance_control_set_enabled(uint8_t enabled)
{
    if (enabled && state.mpu_ok) {
#if BALANCE_ENABLE_BATTERY_PROTECTION
        if ((state.battery_centivolts > 0U) && (state.battery_centivolts < BALANCE_BATTERY_LOW_CV)) {
            stop_with_reason(BALANCE_STOP_LOW_BATTERY);
            return;
        }
#endif
        state.enabled = 1U;
        state.stop_reason = BALANCE_STOP_USER;
        lqr_control_reset_pose();
    } else {
        stop_with_reason(BALANCE_STOP_USER);
    }
}

void balance_control_toggle_enabled(void)
{
    balance_control_set_enabled((uint8_t)!state.enabled);
}

void balance_control_set_mode(balance_mode_t mode)
{
    state.mode = mode;
    lqr_control_reset_pose();
}

void balance_control_set_remote_target(float speed_mps, float yaw_rate_rads, uint8_t last_byte)
{
    remote_speed_target_mps = speed_mps;
    remote_yaw_target_rads = yaw_rate_rads;
    state.remote_last_byte = last_byte;
}

void balance_control_poll(void)
{
    static uint8_t last_key = 1U;
    static uint32_t last_toggle_ms = 0U;
    static uint32_t last_ultrasonic_ms = 0U;
    const uint32_t now = HAL_GetTick();
    const uint8_t key_now = (HAL_GPIO_ReadPin(User_key_GPIO_Port, User_key_Pin) == GPIO_PIN_RESET) ? 0U : 1U;

    if ((last_key == 1U) && (key_now == 0U) && ((now - last_toggle_ms) > 250U)) {
        balance_control_toggle_enabled();
        last_toggle_ms = now;
    }
    last_key = key_now;

    if ((now - last_ultrasonic_ms) >= 80U) {
        ultrasonic_trigger();
        state.ultrasonic_mm = ultrasonic_get_distance_mm();
        last_ultrasonic_ms = now;
    }

    status_display_update();
    HAL_Delay(5U);
}

void balance_control_handle_imu_interrupt(void)
{
    mpu6050_sample_t sample;
    balance_encoder_counts_t counts;
    lqr_state_t lqr;

    if (!state.initialized || !state.mpu_ok) {
        balance_motor_stop();
        return;
    }

    counts = balance_encoder_read();
    state.encoder_left = counts.left_counts;
    state.encoder_right = counts.right_counts;

    if (!mpu6050_raw_read(&sample)) {
        stop_with_reason(BALANCE_STOP_MPU_ERROR);
        state.mpu_ok = 0U;
        return;
    }

    state.pitch_rad = kalman_pitch_update(sample.accel_mps2[1], sample.accel_mps2[2], sample.gyro_rads[0]);
    state.pitch_deg = state.pitch_rad * 180.0f / BALANCE_PI;

    if ((state.control_ticks % 10U) == 0U) {
        state.battery_centivolts = balance_battery_read_centivolts();
        state.ultrasonic_mm = ultrasonic_get_distance_mm();
    }

    if (fabsf(state.pitch_deg) > BALANCE_TILT_LIMIT_DEG) {
        stop_with_reason(BALANCE_STOP_TILT);
    }
#if BALANCE_ENABLE_BATTERY_PROTECTION
    if ((state.battery_centivolts > 0U) && (state.battery_centivolts < BALANCE_BATTERY_LOW_CV)) {
        stop_with_reason(BALANCE_STOP_LOW_BATTERY);
    }
#endif

    apply_control_target();
    lqr = lqr_control_update(counts.left_counts, counts.right_counts, state.pitch_rad);
    state.angle_rate_rads = lqr.angle_rate_rads;
    state.x_pose_m = lqr.x_pose_m;
    state.x_speed_mps = lqr.x_speed_mps;
    state.yaw_rad = lqr.yaw_rad;
    state.yaw_rate_rads = lqr.yaw_rate_rads;
    state.pwm_left = lqr.left_pwm;
    state.pwm_right = lqr.right_pwm;

    if (state.enabled) {
        balance_motor_set(lqr.left_pwm, lqr.right_pwm);
    } else {
        balance_motor_stop();
    }

    state.control_ticks++;
}

void balance_control_get_state(balance_state_t *out_state)
{
    if (out_state == 0) {
        return;
    }
    __disable_irq();
    *out_state = state;
    __enable_irq();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU6050_INT_Pin) {
        balance_control_handle_imu_interrupt();
    }
}
