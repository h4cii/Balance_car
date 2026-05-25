#include "kalman.h"

#include <math.h>

#include "balance_config.h"

#define KALMAN_Q_ANGLE   1.0e-10f
#define KALMAN_Q_BIAS    1.0e-10f
#define KALMAN_R_MEASURE 1.0e-4f

void kalman_init(kalman_filter_t *filter)
{
    if (filter == 0) {
        return;
    }
    filter->angle = 0.0f;
    filter->bias = 0.0f;
    filter->p00 = 1.0f;
    filter->p01 = 0.0f;
    filter->p10 = 0.0f;
    filter->p11 = 1.0f;
}

float kalman_update(kalman_filter_t *filter, float accel_angle_rad, float gyro_rate_rad_s, float dt_s)
{
    float rate;
    float p00_minus;
    float p01_minus;
    float p10_minus;
    float p11_minus;
    float s;
    float k0;
    float k1;
    float y;

    if (filter == 0) {
        return 0.0f;
    }

    rate = gyro_rate_rad_s - filter->bias;
    filter->angle += dt_s * rate;

    p00_minus = filter->p00 - dt_s * (filter->p10 + filter->p01) + (dt_s * dt_s) * filter->p11 + KALMAN_Q_ANGLE;
    p01_minus = filter->p01 - dt_s * filter->p11;
    p10_minus = filter->p10 - dt_s * filter->p11;
    p11_minus = filter->p11 + KALMAN_Q_BIAS;

    s = p00_minus + KALMAN_R_MEASURE;
    k0 = p00_minus / s;
    k1 = p10_minus / s;

    y = accel_angle_rad - filter->angle;
    filter->angle += k0 * y;
    filter->bias += k1 * y;

    filter->p00 = p00_minus - k0 * p00_minus;
    filter->p01 = p01_minus - k0 * p01_minus;
    filter->p10 = p10_minus - k1 * p00_minus;
    filter->p11 = p11_minus - k1 * p01_minus;

    return filter->angle;
}

float kalman_pitch_update(float accel_y_mps2, float accel_z_mps2, float gyro_x_rad_s)
{
    static kalman_filter_t pitch_filter;
    static unsigned char initialized = 0U;
    const float accel_angle = atan2f(-accel_y_mps2, accel_z_mps2);

    if (!initialized) {
        kalman_init(&pitch_filter);
        pitch_filter.angle = accel_angle;
        initialized = 1U;
    }

    return kalman_update(&pitch_filter, accel_angle, -gyro_x_rad_s, BALANCE_SAMPLE_PERIOD_S);
}
