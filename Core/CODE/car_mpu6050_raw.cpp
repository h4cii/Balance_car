#include "car_mpu6050_raw.hpp"

#include "car_soft_i2c.hpp"
#include "main.h"

namespace car {

Mpu6050Raw mpu6050;

void Mpu6050Raw::busInit()
{
    soft_i2c.init();
}

uint8_t Mpu6050Raw::readWhoAmI()
{
    uint8_t who = 0U;
    if (!soft_i2c.readReg(ADDRESS, REG_WHO_AM_I, who)) {
        return 0U;
    }
    return who;
}

bool Mpu6050Raw::init()
{
    uint8_t who = 0U;

    soft_i2c.writeReg(ADDRESS, REG_PWR_MGMT_1, 0x80U);
    HAL_Delay(100U);
    if (!soft_i2c.writeReg(ADDRESS, REG_PWR_MGMT_1, 0x02U)) {
        return false;
    }
    HAL_Delay(10U);

    if (!soft_i2c.readReg(ADDRESS, REG_WHO_AM_I, who) || (who != 0x68U)) {
        return false;
    }

    if (!soft_i2c.writeReg(ADDRESS, REG_SMPLRT_DIV, 0x04U)) {
        return false;
    }
    if (!soft_i2c.writeReg(ADDRESS, REG_CONFIG, 0x03U)) {
        return false;
    }
    if (!soft_i2c.writeReg(ADDRESS, REG_GYRO_CONFIG, 0x18U)) {
        return false;
    }
    if (!soft_i2c.writeReg(ADDRESS, REG_ACCEL_CONFIG, 0x00U)) {
        return false;
    }
    if (!soft_i2c.writeReg(ADDRESS, REG_INT_ENABLE, 0x01U)) {
        return false;
    }
    return true;
}

bool Mpu6050Raw::read(Mpu6050Sample &sample)
{
    uint8_t raw[14];

    if (!soft_i2c.readRegs(ADDRESS, REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        return false;
    }

    sample.accel_raw[0] = be16(&raw[0]);
    sample.accel_raw[1] = be16(&raw[2]);
    sample.accel_raw[2] = be16(&raw[4]);
    const int16_t temp_raw = be16(&raw[6]);
    sample.gyro_raw[0] = be16(&raw[8]);
    sample.gyro_raw[1] = be16(&raw[10]);
    sample.gyro_raw[2] = be16(&raw[12]);

    for (uint8_t i = 0U; i < 3U; ++i) {
        sample.accel_mps2[i] = ((float)sample.accel_raw[i] / ACCEL_LSB_PER_G) * GRAVITY_MPS2;
        sample.gyro_rads[i] = ((float)sample.gyro_raw[i] / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    }
    sample.temperature_c = ((float)temp_raw / 340.0f) + 36.53f;
    return true;
}

int16_t Mpu6050Raw::be16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] << 8U | data[1]);
}

}  // namespace car
