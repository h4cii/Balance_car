#ifndef CAR_CONFIG_HPP
#define CAR_CONFIG_HPP

/* Hardware and controller parameters match the WHEELTEC B570 reference car. */

#define BALANCE_PI                     3.14159265358979323846f
#define BALANCE_SAMPLE_PERIOD_S        0.005f
#define BALANCE_CONTROL_FREQUENCY_HZ   200.0f

#define BALANCE_WHEEL_DIAMETER_MM      67.0f
#define BALANCE_WHEEL_SPACING_MM       161.0f
#define BALANCE_ENCODER_MULTIPLES      4.0f
#define BALANCE_ENCODER_PRECISION      13.0f
#define BALANCE_REDUCTION_RATIO        30.0f
#define BALANCE_ENCODER_COUNTS_PER_REV \
    (BALANCE_ENCODER_MULTIPLES * BALANCE_ENCODER_PRECISION * BALANCE_REDUCTION_RATIO)

#define BALANCE_PWM_MAX                6900
#define BALANCE_PWM_PERIOD             7199

/* Reference LQR coefficients. */
#define BALANCE_LQR_K1                (-77.4597f)
#define BALANCE_LQR_K2                (-113.9570f)
#define BALANCE_LQR_K3                (-357.2249f)
#define BALANCE_LQR_K4                (-33.3211f)
#define BALANCE_LQR_K5                 22.3607f
#define BALANCE_LQR_K6                 22.8301f
#define BALANCE_TARGET_ANGLE_RAD       0.0349f
#define BALANCE_RATIO_ACCEL            5948.0f

/* Reference car uses a 12 V battery divided by 11 before ADC. */
#define BALANCE_BATTERY_DIVIDER        11.0f
#define BALANCE_BATTERY_LOW_CV         1000
#define BALANCE_ENABLE_BATTERY_PROTECTION 1

#define BALANCE_TILT_LIMIT_DEG         40.0f

/* Keep these as the first tuning knobs during bring-up. */
#define BALANCE_LEFT_ENCODER_SIGN      (-1)
#define BALANCE_RIGHT_ENCODER_SIGN     (-1)
#define BALANCE_LEFT_MOTOR_SIGN        (1)
#define BALANCE_RIGHT_MOTOR_SIGN       (1)

#define BALANCE_REMOTE_SPEED_SLOW_MPS  0.30f
#define BALANCE_REMOTE_SPEED_FAST_MPS  0.60f
#define BALANCE_REMOTE_YAW_SLOW_RADS   2.0f
#define BALANCE_REMOTE_YAW_FAST_RADS   4.0f

#define BALANCE_ULTRASONIC_AVOID_MM    450U
#define BALANCE_ULTRASONIC_FOLLOW_NEAR_MM 200U
#define BALANCE_ULTRASONIC_FOLLOW_FAR_MM  500U

#endif
