#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float angle;
    float bias;
    float p00;
    float p01;
    float p10;
    float p11;
} kalman_filter_t;

void kalman_init(kalman_filter_t *filter);
float kalman_update(kalman_filter_t *filter, float accel_angle_rad, float gyro_rate_rad_s, float dt_s);
float kalman_pitch_update(float accel_y_mps2, float accel_z_mps2, float gyro_x_rad_s);

#endif
