#include "car_kalman.hpp"

#include <math.h>

#include "car_config.hpp"

namespace {
constexpr float KALMAN_Q_ANGLE = 1.0e-10f;
constexpr float KALMAN_Q_BIAS = 1.0e-10f;
constexpr float KALMAN_R_MEASURE = 1.0e-4f;
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
                            (dt_s * dt_s) * filter.p11 + KALMAN_Q_ANGLE;
    const float p01_minus = filter.p01 - dt_s * filter.p11;
    const float p10_minus = filter.p10 - dt_s * filter.p11;
    const float p11_minus = filter.p11 + KALMAN_Q_BIAS;

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

float PitchKalmanFilter::update(float accel_y_mps2, float accel_z_mps2, float gyro_x_rad_s)
{
    const float accel_angle = atan2f(-accel_y_mps2, accel_z_mps2);

    if (!initialized_) {
        filter_.init();
        filter_.setAngle(accel_angle);
        initialized_ = true;
    }

    return filter_.update(accel_angle, -gyro_x_rad_s, BALANCE_SAMPLE_PERIOD_S);
}

}  // namespace car
