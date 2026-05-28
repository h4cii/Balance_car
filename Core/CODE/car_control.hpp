#ifndef CAR_CONTROL_HPP
#define CAR_CONTROL_HPP

#include <stdint.h>
#include <math.h>

#include "car_battery.hpp"
#include "car_config.hpp"
#include "car_encoder.hpp"
#include "car_kalman.hpp"
#include "car_lqr_control.hpp"
#include "car_motor.hpp"
#include "car_mpu6050_raw.hpp"
#include "car_remote_control.hpp"
#include "car_time.hpp"
#include "car_ultrasonic.hpp"
#include "main.h"
namespace car {

enum class BalanceMode : uint8_t {
    normal = 0,
    ultrasonicAvoid = 1,
    ultrasonicFollow = 2
};

enum class StopReason : uint8_t {
    user = 0,
    tilt = 1,
    lowBattery = 2,
    mpuError = 3
};

struct BalanceState {
    bool initialized = false;
    bool enabled = false;
    bool mpu_ok = false;
    uint8_t mpu_whoami = 0U;
    BalanceMode mode = BalanceMode::normal;
    StopReason stop_reason = StopReason::user;
    uint32_t control_ticks = 0U;
    float pitch_rad = 0.0f;
    float pitch_deg = 0.0f;
    float angle_rate_rads = 0.0f;
    float x_pose_m = 0.0f;
    float x_speed_mps = 0.0f;
    float yaw_rad = 0.0f;
    float yaw_rate_rads = 0.0f;
    uint16_t ultrasonic_mm = 0U;
    uint16_t battery_centivolts = 0U;
    int16_t encoder_left = 0;
    int16_t encoder_right = 0;
    int16_t pwm_left = 0;
    int16_t pwm_right = 0;
    uint8_t remote_last_byte = 0U;
};

class BalanceController final {
public:
    void init();
    void poll();
    void handleImuInterrupt();
    void setEnabled(bool enabled);
    void toggleEnabled();
    void setMode(BalanceMode mode);
    void setRemoteTarget(float speed_mps, float yaw_rate_rads, uint8_t last_byte);
    BalanceState state();

private:
    void stopWithReason(StopReason reason);
    void applyControlTarget();

    BalanceState state_ {};
    float remote_speed_target_mps_ = 0.0f;
    float remote_yaw_target_rads_ = 0.0f;
    bool last_key_ = true;
    uint32_t last_toggle_ms_ = 0U;
    uint32_t last_ultrasonic_ms_ = 0U;
};

extern BalanceController controller;

}  // namespace car

#endif
