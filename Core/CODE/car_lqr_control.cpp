#include "car_lqr_control.hpp"

#include "car_config.hpp"
#include "car_motor.hpp"

namespace car {

LqrController lqr;

void LqrController::init()
{
    state_ = {};
    target_speed_mps_ = 0.0f;
    target_yaw_rate_rads_ = 0.0f;
    last_pitch_rad_ = 0.0f;
}

void LqrController::resetPose()
{
    state_.x_pose_m = 0.0f;
    state_.yaw_rad = 0.0f;
}

void LqrController::setTarget(float speed_mps, float yaw_rate_rads)
{
    if ((speed_mps != target_speed_mps_) || (yaw_rate_rads != target_yaw_rate_rads_)) {
        resetPose();
    }
    target_speed_mps_ = speed_mps;
    target_yaw_rate_rads_ = yaw_rate_rads;
}

LqrState LqrController::update(int16_t left_counts, int16_t right_counts, float pitch_rad)
{
    const float wheel_circ_m = BALANCE_PI * BALANCE_WHEEL_DIAMETER_MM / 1000.0f;
    const float track_m = BALANCE_WHEEL_SPACING_MM / 1000.0f;
    const float average_counts = ((float)left_counts + (float)right_counts) * 0.5f;
    // const float diff_counts = (float)right_counts - (float)left_counts;
    const float diff_counts = (float)left_counts - (float)right_counts;
    state_.x_speed_mps = average_counts * wheel_circ_m * BALANCE_CONTROL_FREQUENCY_HZ /
                         BALANCE_ENCODER_COUNTS_PER_REV;
    state_.x_pose_m += state_.x_speed_mps * BALANCE_SAMPLE_PERIOD_S;

    state_.angle_rad = pitch_rad;
    state_.angle_rate_rads = (pitch_rad - last_pitch_rad_) * BALANCE_CONTROL_FREQUENCY_HZ;
    last_pitch_rad_ = pitch_rad;

    state_.yaw_rate_rads = diff_counts * wheel_circ_m * BALANCE_CONTROL_FREQUENCY_HZ /
                           (BALANCE_ENCODER_COUNTS_PER_REV * track_m);
    state_.yaw_rad += state_.yaw_rate_rads * BALANCE_SAMPLE_PERIOD_S;

    state_.left_accel = -(BALANCE_LQR_K1 * state_.x_pose_m +
                          BALANCE_LQR_K2 * (state_.x_speed_mps - target_speed_mps_) +
                          BALANCE_LQR_K3 * (state_.angle_rad - BALANCE_TARGET_ANGLE_RAD) +
                          BALANCE_LQR_K4 * state_.angle_rate_rads +
                          BALANCE_LQR_K5 * state_.yaw_rad +
                          BALANCE_LQR_K6 * (state_.yaw_rate_rads - target_yaw_rate_rads_));

    state_.right_accel = -(BALANCE_LQR_K1 * state_.x_pose_m +
                           BALANCE_LQR_K2 * (state_.x_speed_mps - target_speed_mps_) +
                           BALANCE_LQR_K3 * (state_.angle_rad - BALANCE_TARGET_ANGLE_RAD) +
                           BALANCE_LQR_K4 * state_.angle_rate_rads -
                           BALANCE_LQR_K5 * state_.yaw_rad -
                           BALANCE_LQR_K6 * (state_.yaw_rate_rads - target_yaw_rate_rads_));

    state_.left_pwm = pwmFromFloat(BALANCE_RATIO_ACCEL *
                                   (state_.x_speed_mps + state_.left_accel * BALANCE_SAMPLE_PERIOD_S));
    state_.right_pwm = pwmFromFloat(BALANCE_RATIO_ACCEL *
                                    (state_.x_speed_mps + state_.right_accel * BALANCE_SAMPLE_PERIOD_S));
    return state_;
}

LqrState LqrController::state() const
{
    return state_;
}

int16_t LqrController::pwmFromFloat(float value)
{
    if (value >= 0.0f) {
        return MotorDriver::limit((int32_t)(value + 0.5f));
    }
    return MotorDriver::limit((int32_t)(value - 0.5f));
}

}  // namespace car
