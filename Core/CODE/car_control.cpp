#include "car_control.hpp"

#include <math.h>

#include "car_battery.hpp"
#include "car_config.hpp"
#include "car_encoder.hpp"
#include "car_kalman.hpp"
#include "car_lqr_control.hpp"
#include "car_motor.hpp"
#include "car_mpu6050_raw.hpp"
#include "car_remote_control.hpp"
#include "car_status_display.hpp"
#include "car_time.hpp"
#include "car_ultrasonic.hpp"
#include "main.h"

namespace car {

BalanceController controller;

void BalanceController::init()
{
    state_ = {};
    state_.stop_reason = StopReason::user;
    state_.mode = BalanceMode::normal;

    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(MPU6050_INT_Pin);

    micro_timer.init();
    motor.init();
    encoder.init();
    battery.init();
    lqr.init();
    mpu6050.busInit();
    ultrasonic.init();
    remote.init();
    status_display.init();

    state_.battery_centivolts = battery.readCentivolts();
    state_.mpu_whoami = mpu6050.readWhoAmI();
    state_.mpu_ok = mpu6050.init();
    if (!state_.mpu_ok) {
        stopWithReason(StopReason::mpuError);
    }

    state_.initialized = true;
    if (state_.mpu_ok) {
        __HAL_GPIO_EXTI_CLEAR_IT(MPU6050_INT_Pin);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
}

void BalanceController::setEnabled(bool enabled)
{
    if (enabled && state_.mpu_ok) {
#if BALANCE_ENABLE_BATTERY_PROTECTION
        if ((state_.battery_centivolts > 0U) && (state_.battery_centivolts < BALANCE_BATTERY_LOW_CV)) {
            stopWithReason(StopReason::lowBattery);
            return;
        }
#endif
        state_.enabled = true;
        state_.stop_reason = StopReason::user;
        lqr.resetPose();
    } else {
        stopWithReason(StopReason::user);
    }
}

void BalanceController::toggleEnabled()
{
    setEnabled(!state_.enabled);
}

void BalanceController::setMode(BalanceMode mode)
{
    state_.mode = mode;
    lqr.resetPose();
}

void BalanceController::setRemoteTarget(float speed_mps, float yaw_rate_rads, uint8_t last_byte)
{
    remote_speed_target_mps_ = speed_mps;
    remote_yaw_target_rads_ = yaw_rate_rads;
    state_.remote_last_byte = last_byte;
}

void BalanceController::poll()
{
    const uint32_t now = HAL_GetTick();
    const bool key_now = HAL_GPIO_ReadPin(User_key_GPIO_Port, User_key_Pin) != GPIO_PIN_RESET;

    if (last_key_ && !key_now && ((now - last_toggle_ms_) > 250U)) {
        toggleEnabled();
        last_toggle_ms_ = now;
    }
    last_key_ = key_now;

    if ((now - last_ultrasonic_ms_) >= 80U) {
        ultrasonic.trigger();
        state_.ultrasonic_mm = ultrasonic.distanceMm();
        last_ultrasonic_ms_ = now;
    }

    status_display.update();
    HAL_Delay(5U);
}

void BalanceController::handleImuInterrupt()
{
    if (!state_.initialized || !state_.mpu_ok) {
        motor.stop();
        return;
    }

    const EncoderCounts counts = encoder.read();
    state_.encoder_left = counts.left_counts;
    state_.encoder_right = counts.right_counts;

    Mpu6050Sample sample {};
    if (!mpu6050.read(sample)) {
        stopWithReason(StopReason::mpuError);
        state_.mpu_ok = false;
        return;
    }

    state_.pitch_rad = pitch_kalman.update(sample.accel_mps2[1], sample.accel_mps2[2], sample.gyro_rads[0]);
    state_.pitch_deg = state_.pitch_rad * 180.0f / BALANCE_PI;

    if ((state_.control_ticks % 10U) == 0U) {
        state_.battery_centivolts = battery.readCentivolts();
        state_.ultrasonic_mm = ultrasonic.distanceMm();
    }

    if (fabsf(state_.pitch_deg) > BALANCE_TILT_LIMIT_DEG) {
        stopWithReason(StopReason::tilt);
    }
#if BALANCE_ENABLE_BATTERY_PROTECTION
    if ((state_.battery_centivolts > 0U) && (state_.battery_centivolts < BALANCE_BATTERY_LOW_CV)) {
        stopWithReason(StopReason::lowBattery);
    }
#endif

    applyControlTarget();
    const LqrState lqr_state = lqr.update(counts.left_counts, counts.right_counts, state_.pitch_rad);
    state_.angle_rate_rads = lqr_state.angle_rate_rads;
    state_.x_pose_m = lqr_state.x_pose_m;
    state_.x_speed_mps = lqr_state.x_speed_mps;
    state_.yaw_rad = lqr_state.yaw_rad;
    state_.yaw_rate_rads = lqr_state.yaw_rate_rads;
    state_.pwm_left = lqr_state.left_pwm;
    state_.pwm_right = lqr_state.right_pwm;

    if (state_.enabled) {
        motor.set(lqr_state.left_pwm, lqr_state.right_pwm);
    } else {
        motor.stop();
    }

    state_.control_ticks++;
}

BalanceState BalanceController::state()
{
    __disable_irq();
    const BalanceState snapshot = state_;
    __enable_irq();
    return snapshot;
}

void BalanceController::stopWithReason(StopReason reason)
{
    state_.enabled = false;
    state_.stop_reason = reason;
    remote_speed_target_mps_ = 0.0f;
    remote_yaw_target_rads_ = 0.0f;
    lqr.setTarget(0.0f, 0.0f);
    lqr.resetPose();
    motor.stop();
}

void BalanceController::applyControlTarget()
{
    float speed = remote_speed_target_mps_;
    float yaw_rate = remote_yaw_target_rads_;

    if (state_.mode == BalanceMode::ultrasonicAvoid) {
        if ((state_.ultrasonic_mm > 0U) && (state_.ultrasonic_mm < BALANCE_ULTRASONIC_AVOID_MM)) {
            speed = -BALANCE_REMOTE_SPEED_SLOW_MPS;
            yaw_rate = 0.0f;
        }
    } else if (state_.mode == BalanceMode::ultrasonicFollow) {
        if ((state_.ultrasonic_mm > BALANCE_ULTRASONIC_FOLLOW_FAR_MM) &&
            (state_.ultrasonic_mm < 1500U)) {
            speed = BALANCE_REMOTE_SPEED_SLOW_MPS;
        } else if ((state_.ultrasonic_mm > 0U) &&
                   (state_.ultrasonic_mm < BALANCE_ULTRASONIC_FOLLOW_NEAR_MM)) {
            speed = -BALANCE_REMOTE_SPEED_SLOW_MPS;
        } else {
            speed = 0.0f;
        }
        yaw_rate = 0.0f;
    }

    lqr.setTarget(speed, yaw_rate);
}

}  // namespace car

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU6050_INT_Pin) {
        car::controller.handleImuInterrupt();
    }
}
