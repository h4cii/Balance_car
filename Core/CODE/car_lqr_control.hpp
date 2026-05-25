#ifndef CAR_LQR_CONTROL_HPP
#define CAR_LQR_CONTROL_HPP

#include <stdint.h>

namespace car {

struct LqrState {
    float x_pose_m;
    float x_speed_mps;
    float angle_rad;
    float angle_rate_rads;
    float yaw_rad;
    float yaw_rate_rads;
    float left_accel;
    float right_accel;
    int16_t left_pwm;
    int16_t right_pwm;
};

class LqrController final {
public:
    void init();
    void resetPose();
    void setTarget(float speed_mps, float yaw_rate_rads);
    LqrState update(int16_t left_counts, int16_t right_counts, float pitch_rad);
    LqrState state() const;

private:
    static int16_t pwmFromFloat(float value);

    LqrState state_ {};
    float target_speed_mps_ = 0.0f;
    float target_yaw_rate_rads_ = 0.0f;
    float last_pitch_rad_ = 0.0f;
};

extern LqrController lqr;

}  // namespace car

#endif
