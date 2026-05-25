#ifndef CAR_KALMAN_HPP
#define CAR_KALMAN_HPP

namespace car {

struct KalmanState {
    float angle;
    float bias;
    float p00;
    float p01;
    float p10;
    float p11;
};

class KalmanFilter final {
public:
    void init();
    void setAngle(float angle_rad);
    float update(float accel_angle_rad, float gyro_rate_rad_s, float dt_s);

private:
    static float updateState(KalmanState &filter,
                             float accel_angle_rad,
                             float gyro_rate_rad_s,
                             float dt_s);

    KalmanState data_ {};
};

class PitchKalmanFilter final {
public:
    float update(float accel_y_mps2, float accel_z_mps2, float gyro_x_rad_s);

private:
    KalmanFilter filter_ {};
    bool initialized_ = false;
};

extern PitchKalmanFilter pitch_kalman;

}  // namespace car

#endif
