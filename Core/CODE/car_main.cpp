#include "car_main.hpp"

#include "car_app_scheduler.hpp"
#include "car_battery.hpp"
#include "car_config.hpp"
#include "car_control.hpp"
#include "car_encoder.hpp"
#include "car_kalman.hpp"
#include "car_lqr_control.hpp"
#include "car_motor.hpp"
#include "car_mpu6050_raw.hpp"
#include "car_oled_ui.hpp"
#include "car_remote_control.hpp"
#include "car_time.hpp"
#include "car_ultrasonic.hpp"
#include "main.h"

#define CAR_APP_ENABLE_OLED_UI     1
#define CAR_APP_ENABLE_BATTERY     0
#define CAR_APP_ENABLE_MPU6050     1
#define CAR_APP_ENABLE_ENCODER     1
#define CAR_APP_ENABLE_MOTOR       1
#define CAR_APP_ENABLE_MOTOR_TEST  0
#define CAR_APP_ENABLE_ULTRASONIC  0
#define CAR_APP_ENABLE_REMOTE      0
#define CAR_APP_ENABLE_CONTROLLER  1

#define CAR_APP_MOTOR_TEST_PWM     1000

namespace {

struct AppRuntime {
    car::OledUiData ui {};
    uint32_t last_battery_ms = 0U;
    uint32_t last_mpu_ms = 0U;
    uint32_t last_encoder_ms = 0U;
    uint32_t last_ultrasonic_ms = 0U;
};

int16_t pitchDegX10(float pitch_rad)
{
    const float value = pitch_rad * 1800.0f / BALANCE_PI;
    if (value >= 0.0f) {
        return (int16_t)(value + 0.5f);
    }
    return (int16_t)(value - 0.5f);
}

void initModules(AppRuntime &app)
{
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    __HAL_GPIO_EXTI_CLEAR_IT(MPU6050_INT_Pin);

#if CAR_APP_ENABLE_OLED_UI
    car::oled_ui_demo.init();
#endif

#if CAR_APP_ENABLE_MOTOR
    car::motor.init();
#if CAR_APP_ENABLE_MOTOR_TEST && !CAR_APP_ENABLE_CONTROLLER
    car::motor.set(CAR_APP_MOTOR_TEST_PWM, CAR_APP_MOTOR_TEST_PWM);
    app.ui.enabled = true;
    app.ui.pwm_left = CAR_APP_MOTOR_TEST_PWM;
    app.ui.pwm_right = CAR_APP_MOTOR_TEST_PWM;
#endif
#endif

#if CAR_APP_ENABLE_ENCODER
    car::encoder.init();
#endif

#if CAR_APP_ENABLE_BATTERY
    car::battery.init();
    app.ui.battery_centivolts = car::battery.readCentivolts();
#endif

#if CAR_APP_ENABLE_MPU6050
    car::mpu6050.busInit();
    app.ui.mpu_ok = car::mpu6050.init();
#endif

#if CAR_APP_ENABLE_CONTROLLER
    car::lqr.init();
#endif

#if CAR_APP_ENABLE_ULTRASONIC
    car::micro_timer.init();
    car::ultrasonic.init();
#endif

#if CAR_APP_ENABLE_REMOTE
    car::remote.init();
#endif

#if CAR_APP_ENABLE_CONTROLLER
    car::controller.init();
#endif
}

void updateStandaloneData(AppRuntime &app)
{
    const uint32_t now = HAL_GetTick();

#if CAR_APP_ENABLE_BATTERY
    if ((now - app.last_battery_ms) >= 500U) {
        app.ui.battery_centivolts = car::battery.readCentivolts();
        app.last_battery_ms = now;
    }
#endif

#if CAR_APP_ENABLE_MPU6050
    if (app.ui.mpu_ok) {
        car::Mpu6050Sample sample {};
        if (car::mpu6050.read(sample)) {
            float dt_s = BALANCE_SAMPLE_PERIOD_S;
            if (app.last_mpu_ms != 0U) {
                dt_s = (float)(now - app.last_mpu_ms) / 1000.0f;
            }
            const float pitch_rad = car::pitch_kalman.update(sample.accel_mps2[1],
                                                             sample.accel_mps2[2],
                                                             sample.gyro_rads[0],
                                                             dt_s);
            app.ui.pitch_deg_x10 = pitchDegX10(pitch_rad);
        } else {
            app.ui.mpu_ok = false;
        }
        app.last_mpu_ms = now;
    }
#endif

#if CAR_APP_ENABLE_ENCODER
    if ((now - app.last_encoder_ms) >= 50U) {
        const car::EncoderCounts counts = car::encoder.read();
        app.ui.encoder_left = counts.left_counts;
        app.ui.encoder_right = counts.right_counts;
        app.last_encoder_ms = now;
    }
#endif

#if CAR_APP_ENABLE_ULTRASONIC
    if (car::app_scheduler.takeUltrasonicTick()) {
        car::ultrasonic.trigger();
        app.last_ultrasonic_ms = now;
    }
#endif
}

void updateControllerData(AppRuntime &app)
{
#if CAR_APP_ENABLE_CONTROLLER
    car::controller.poll();
    const car::BalanceState s = car::controller.state();
    app.ui.enabled = s.enabled;
    app.ui.mpu_ok = s.mpu_ok;
    app.ui.battery_centivolts = s.battery_centivolts;
    app.ui.pitch_deg_x10 = pitchDegX10(s.pitch_rad);
    app.ui.pwm_left = s.pwm_left;
    app.ui.pwm_right = s.pwm_right;
    app.ui.encoder_left = s.encoder_left;
    app.ui.encoder_right = s.encoder_right;
#else
    updateStandaloneData(app);
#endif
}

}  // namespace

extern "C" void my_main(void)
{
    AppRuntime app {};

    initModules(app);
    car::app_scheduler.start();

    for (;;) {
        if (car::app_scheduler.takeImuTick()) {
            updateControllerData(app);
        }
#if CAR_APP_ENABLE_OLED_UI
        if (car::app_scheduler.takeOledTick()) {
            car::oled_ui_demo.update(app.ui);
        }
#endif
        HAL_Delay(1U);
    }
}
