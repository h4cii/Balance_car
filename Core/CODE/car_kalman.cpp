#include "car_kalman.hpp"

#include <math.h>

#include "car_config.hpp"

namespace {
constexpr float KALMAN_Q_ANGLE = 0.001f;
constexpr float KALMAN_Q_BIAS = 0.003f;
constexpr float KALMAN_R_MEASURE = 0.5f;
constexpr uint16_t GYRO_BIAS_SAMPLE_COUNT = 100U;
}  // namespace

namespace car {

PitchKalmanFilter pitch_kalman;

void KalmanFilter::init()
{
    data_ = {
        0.0f,
        0.0f,
        1.0f,
        0.0f,
        0.0f,
        1.0f,
    };
}

void KalmanFilter::setAngle(float angle_rad)
{
    data_.angle = angle_rad;
}

float KalmanFilter::update(float accel_angle_rad, float gyro_rate_rad_s, float dt_s)
{
    return updateState(data_, accel_angle_rad, gyro_rate_rad_s, dt_s);
}

float KalmanFilter::updateState(KalmanState &filter,
                                float accel_angle_rad,
                                float gyro_rate_rad_s,
                                float dt_s)
{
    const float rate = gyro_rate_rad_s - filter.bias;
    filter.angle += dt_s * rate;

    const float p00_minus = filter.p00 - dt_s * (filter.p10 + filter.p01) +
                            (dt_s * dt_s) * filter.p11 + (KALMAN_Q_ANGLE * dt_s);
    const float p01_minus = filter.p01 - dt_s * filter.p11;
    const float p10_minus = filter.p10 - dt_s * filter.p11;
    const float p11_minus = filter.p11 + (KALMAN_Q_BIAS * dt_s);

    const float s = p00_minus + KALMAN_R_MEASURE;
    const float k0 = p00_minus / s;
    const float k1 = p10_minus / s;

    const float y = accel_angle_rad - filter.angle;
    filter.angle += k0 * y;
    filter.bias += k1 * y;

    filter.p00 = p00_minus - k0 * p00_minus;
    filter.p01 = p01_minus - k0 * p01_minus;
    filter.p10 = p10_minus - k1 * p00_minus;
    filter.p11 = p11_minus - k1 * p01_minus;

    return filter.angle;
}

float PitchKalmanFilter::update(float accel_y_mps2, float accel_z_mps2, float gyro_x_rad_s, float dt_s)
{
    const float accel_angle = atan2f(-accel_y_mps2, accel_z_mps2);

    if (!initialized_) {
        filter_.init();
        filter_.setAngle(accel_angle);
        angle_rad_ = accel_angle;
        initialized_ = true;
    }

    if (dt_s < 0.001f) {
        dt_s = 0.001f;
    } else if (dt_s > 0.05f) {
        dt_s = 0.05f;
    }

    const float gyro_rate = -gyro_x_rad_s;
    if (!gyro_bias_ready_) {
        gyro_bias_sum_ += gyro_rate;
        gyro_bias_samples_++;
        angle_rad_ = (0.65f * angle_rad_) + (0.35f * accel_angle);
        if (gyro_bias_samples_ >= GYRO_BIAS_SAMPLE_COUNT) {
            gyro_bias_rad_s_ = gyro_bias_sum_ / (float)gyro_bias_samples_;
            gyro_bias_ready_ = true;
            filter_.init();
            filter_.setAngle(angle_rad_);
        }
        return angle_rad_;
    }

    const float corrected_gyro = gyro_rate - gyro_bias_rad_s_;
    angle_rad_ = filter_.update(accel_angle, corrected_gyro, dt_s);
    return angle_rad_;
}

}  // namespace car
