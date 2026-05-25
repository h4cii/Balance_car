#ifndef CAR_MPU6050_RAW_HPP
#define CAR_MPU6050_RAW_HPP

#include <stdint.h>

namespace car {

struct Mpu6050Sample {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    float accel_mps2[3];
    float gyro_rads[3];
    float temperature_c;
};

class Mpu6050Raw final {
public:
    void busInit();
    bool init();
    uint8_t readWhoAmI();
    bool read(Mpu6050Sample &sample);

private:
    static constexpr uint8_t ADDRESS = 0x68U;
    static constexpr uint8_t REG_SMPLRT_DIV = 0x19U;
    static constexpr uint8_t REG_CONFIG = 0x1AU;
    static constexpr uint8_t REG_GYRO_CONFIG = 0x1BU;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1CU;
    static constexpr uint8_t REG_INT_ENABLE = 0x38U;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3BU;
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6BU;
    static constexpr uint8_t REG_WHO_AM_I = 0x75U;

    static constexpr float GRAVITY_MPS2 = 9.80665f;
    static constexpr float ACCEL_LSB_PER_G = 16384.0f;
    static constexpr float GYRO_LSB_PER_DPS = 16.4f;
    static constexpr float DEG_TO_RAD = 0.017453292519943295f;

    static int16_t be16(const uint8_t *data);
};

extern Mpu6050Raw mpu6050;

}  // namespace car

#endif
