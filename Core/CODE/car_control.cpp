#include "car_control.hpp"


namespace car {

BalanceController controller;

namespace {

float clampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

float moveToward(float value, float target, float step)
{
    if (value < target) {
        return ((target - value) > step) ? (value + step) : target;
    }
    if (value > target) {
        return ((value - target) > step) ? (value - step) : target;
    }
    return value;
}

float smoothStep(float value)
{
    const float x = clampFloat(value, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

bool isStopByte(uint8_t byte)
{
    return (byte == 'Z') || (byte == 'z') || (byte == 0x00U);
}

bool hasManualSpeed(float speed_mps)
{
    return fabsf(speed_mps) > 0.001f;
}

float followSpeedFromDistance(uint16_t distance_mm)
{
    if (distance_mm > BALANCE_ULTRASONIC_FOLLOW_FAR_MM) {
        return 0.0f;
    }

    const float error_mm = (float)distance_mm -
                           (float)BALANCE_ULTRASONIC_FOLLOW_TARGET_MM;
    const float deadband_mm = (float)BALANCE_ULTRASONIC_FOLLOW_DEADBAND_MM;
    if (fabsf(error_mm) <= deadband_mm) {
        return 0.0f;
    }

    if (error_mm > 0.0f) {
        const float range_mm = (float)BALANCE_ULTRASONIC_FOLLOW_FAR_MM -
                               (float)BALANCE_ULTRASONIC_FOLLOW_TARGET_MM -
                               deadband_mm;
        const float ratio = (error_mm - deadband_mm) / clampFloat(range_mm, 1.0f, 10000.0f);
        return BALANCE_ULTRASONIC_FOLLOW_FORWARD_MAX_MPS * smoothStep(ratio);
    }

    const float range_mm = (float)BALANCE_ULTRASONIC_FOLLOW_TARGET_MM -
                           (float)BALANCE_ULTRASONIC_FOLLOW_NEAR_MM -
                           deadband_mm;
    const float ratio = (-error_mm - deadband_mm) / clampFloat(range_mm, 1.0f, 10000.0f);
    return -BALANCE_ULTRASONIC_FOLLOW_REVERSE_MAX_MPS * smoothStep(ratio);
}

}  // namespace

void BalanceController::init()
{
    state_ = {};
    state_.stop_reason = StopReason::user;
    state_.mode = BalanceMode::normal;

    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(MPU6050_INT_Pin);

#if BALANCE_ENABLE_BATTERY_MONITOR
    state_.battery_centivolts = battery.readCentivolts();
#else
    state_.battery_centivolts = 0U;
#endif
    state_.mpu_whoami = mpu6050.readWhoAmI();
    state_.mpu_ok = (state_.mpu_whoami == 0x68U);
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
    ultrasonic_auto_speed_mps_ = 0.0f;
    ultrasonic_avoid_active_ = false;
    ultrasonic_auto_paused_ = false;
    lqr.resetPose();
}

void BalanceController::cycleMode()
{
    switch (state_.mode) {
    case BalanceMode::normal:
        setMode(BalanceMode::ultrasonicAvoid);
        break;
    case BalanceMode::ultrasonicAvoid:
        setMode(BalanceMode::ultrasonicFollow);
        break;
    default:
        setMode(BalanceMode::normal);
        break;
    }
}

void BalanceController::setRemoteTarget(float speed_mps, float yaw_rate_rads, uint8_t last_byte)
{
    remote_speed_target_mps_ = speed_mps;
    remote_yaw_target_rads_ = yaw_rate_rads;
    state_.remote_last_byte = last_byte;
    if (isStopByte(last_byte)) {
        ultrasonic_auto_paused_ = true;
        ultrasonic_auto_speed_mps_ = 0.0f;
    } else if ((speed_mps != 0.0f) || (yaw_rate_rads != 0.0f)) {
        ultrasonic_auto_paused_ = false;
    }
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
        refreshUltrasonicState();
        last_ultrasonic_ms_ = now;
    }

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

    state_.pitch_rad = pitch_kalman.update(sample.accel_mps2[1],
                                           sample.accel_mps2[2],
                                           sample.gyro_rads[0],
                                           BALANCE_SAMPLE_PERIOD_S);
    state_.pitch_deg = state_.pitch_rad * 180.0f / BALANCE_PI;

    if ((state_.control_ticks % 10U) == 0U) {
#if BALANCE_ENABLE_BATTERY_MONITOR
        state_.battery_centivolts = battery.readCentivolts();
#else
        state_.battery_centivolts = 0U;
#endif
        refreshUltrasonicState();
    }

    if (fabsf(state_.pitch_deg) > BALANCE_TILT_LIMIT_DEG) {
        stopWithReason(StopReason::tilt);
    }
#if BALANCE_ENABLE_BATTERY_PROTECTION
    if ((state_.battery_centivolts > 0U) && (state_.battery_centivolts < BALANCE_BATTERY_LOW_CV)) {
        stopWithReason(StopReason::lowBattery);
    }
#endif
    if (!state_.enabled) {
        motor.stop();
        state_.pwm_left = 0;
        state_.pwm_right = 0;
        lqr.resetPose();
        state_.control_ticks++;
        return;
    }
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
    ultrasonic_auto_speed_mps_ = 0.0f;
    ultrasonic_avoid_active_ = false;
    ultrasonic_auto_paused_ = false;
    lqr.setTarget(0.0f, 0.0f);
    lqr.resetPose();
    motor.stop();
}

void BalanceController::refreshUltrasonicState()
{
    const uint32_t now = HAL_GetTick();
    const bool valid = ultrasonic.hasFreshDistance(now);
    if (valid) {
        const float raw_mm = (float)ultrasonic.distanceMm();
        if (!ultrasonic_filter_ready_) {
            ultrasonic_filtered_mm_ = raw_mm;
            ultrasonic_filter_ready_ = true;
        } else {
            ultrasonic_filtered_mm_ += (raw_mm - ultrasonic_filtered_mm_) *
                                       BALANCE_ULTRASONIC_FILTER_ALPHA;
        }
        state_.ultrasonic_mm = (uint16_t)(ultrasonic_filtered_mm_ + 0.5f);
    } else {
        ultrasonic_filter_ready_ = false;
        state_.ultrasonic_mm = 0U;
    }
    state_.ultrasonic_valid = valid;
}

void BalanceController::applyControlTarget()
{
    float speed = remote_speed_target_mps_;
    float yaw_rate = remote_yaw_target_rads_;
    const bool manual_speed = hasManualSpeed(remote_speed_target_mps_);
    bool ultrasonic_auto = false;

    if (state_.mode == BalanceMode::ultrasonicAvoid) {
        yaw_rate *= BALANCE_ULTRASONIC_FOLLOW_YAW_SCALE;
        if (!state_.ultrasonic_valid) {
            ultrasonic_avoid_active_ = false;
        } else if (state_.ultrasonic_mm < BALANCE_ULTRASONIC_AVOID_MM) {
            ultrasonic_avoid_active_ = true;
        } else if (state_.ultrasonic_mm > BALANCE_ULTRASONIC_AVOID_RELEASE_MM) {
            ultrasonic_avoid_active_ = false;
        }

        if (ultrasonic_avoid_active_) {
            const float error_mm = (float)BALANCE_ULTRASONIC_AVOID_RELEASE_MM -
                                   (float)state_.ultrasonic_mm;
            const float auto_speed = -clampFloat(error_mm * BALANCE_ULTRASONIC_AVOID_SPEED_GAIN_MPS_PER_MM,
                                                 0.0f,
                                                 BALANCE_ULTRASONIC_AUTO_SPEED_MAX_MPS);
            if (manual_speed && (remote_speed_target_mps_ < 0.0f)) {
                speed = remote_speed_target_mps_ * BALANCE_ULTRASONIC_MANUAL_SPEED_SCALE;
            } else {
                speed = auto_speed;
                ultrasonic_auto = true;
            }
        } else if (manual_speed) {
            speed = remote_speed_target_mps_ * BALANCE_ULTRASONIC_MANUAL_SPEED_SCALE;
        }
    } else if (state_.mode == BalanceMode::ultrasonicFollow) {
        yaw_rate *= BALANCE_ULTRASONIC_FOLLOW_YAW_SCALE;
        if (manual_speed) {
            speed = remote_speed_target_mps_ * BALANCE_ULTRASONIC_MANUAL_SPEED_SCALE;
        } else if (ultrasonic_auto_paused_ || !state_.ultrasonic_valid) {
            speed = 0.0f;
            ultrasonic_auto = true;
        } else {
            speed = followSpeedFromDistance(state_.ultrasonic_mm);
            ultrasonic_auto = true;
        }
    }

    if (ultrasonic_auto) {
        const float step = BALANCE_ULTRASONIC_SPEED_SLEW_MPS2 * BALANCE_SAMPLE_PERIOD_S;
        ultrasonic_auto_speed_mps_ = moveToward(ultrasonic_auto_speed_mps_, speed, step);
        speed = ultrasonic_auto_speed_mps_;
    } else {
        ultrasonic_auto_speed_mps_ = speed;
    }

    lqr.setTarget(speed, yaw_rate);
}

}  // namespace car

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU6050_INT_Pin) {
        car::controller.handleImuInterrupt();
    } else if ((GPIO_Pin == UltrasonicCapture_Pin) || (GPIO_Pin == UltrasonicCaptureAlt_Pin)) {
        car::ultrasonic.onEchoEdge(GPIO_Pin);
    }
}
