#include "mpu6050_raw.h"

#include "balance_time.h"
#include "main.h"
#include "soft_i2c.h"

#define MPU6050_ADDR              0x68U
#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_INT_ENABLE    0x38U
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U

#define MPU6050_GRAVITY_MPS2      9.80665f
#define MPU6050_ACCEL_LSB_PER_G   16384.0f
#define MPU6050_GYRO_LSB_PER_DPS  16.4f
#define DEG_TO_RAD                0.017453292519943295f

static int16_t be16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] << 8U | data[1]);
}

void mpu6050_raw_bus_init(void)
{
    soft_i2c_init();
}

uint8_t mpu6050_raw_read_whoami(void)
{
    uint8_t who = 0U;
    if (!soft_i2c_read_reg(MPU6050_ADDR, MPU6050_REG_WHO_AM_I, &who)) {
        return 0U;
    }
    return who;
}

uint8_t mpu6050_raw_init(void)
{
    uint8_t who = 0U;

    soft_i2c_write_reg(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x80U);
    HAL_Delay(100U);
    if (!soft_i2c_write_reg(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x02U)) {
        return 0U;
    }
    HAL_Delay(10U);

    if (!soft_i2c_read_reg(MPU6050_ADDR, MPU6050_REG_WHO_AM_I, &who) || (who != 0x68U)) {
        return 0U;
    }

    if (!soft_i2c_write_reg(MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV, 0x04U)) {
        return 0U;
    }
    if (!soft_i2c_write_reg(MPU6050_ADDR, MPU6050_REG_CONFIG, 0x03U)) {
        return 0U;
    }
    if (!soft_i2c_write_reg(MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, 0x18U)) {
        return 0U;
    }
    if (!soft_i2c_write_reg(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 0x00U)) {
        return 0U;
    }
    if (!soft_i2c_write_reg(MPU6050_ADDR, MPU6050_REG_INT_ENABLE, 0x01U)) {
        return 0U;
    }
    return 1U;
}

uint8_t mpu6050_raw_read(mpu6050_sample_t *sample)
{
    uint8_t raw[14];
    int16_t temp_raw;

    if (sample == 0) {
        return 0U;
    }
    if (!soft_i2c_read_regs(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        return 0U;
    }

    sample->accel_raw[0] = be16(&raw[0]);
    sample->accel_raw[1] = be16(&raw[2]);
    sample->accel_raw[2] = be16(&raw[4]);
    temp_raw = be16(&raw[6]);
    sample->gyro_raw[0] = be16(&raw[8]);
    sample->gyro_raw[1] = be16(&raw[10]);
    sample->gyro_raw[2] = be16(&raw[12]);

    for (uint8_t i = 0U; i < 3U; ++i) {
        sample->accel_mps2[i] = ((float)sample->accel_raw[i] / MPU6050_ACCEL_LSB_PER_G) * MPU6050_GRAVITY_MPS2;
        sample->gyro_rads[i] = ((float)sample->gyro_raw[i] / MPU6050_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    }
    sample->temperature_c = ((float)temp_raw / 340.0f) + 36.53f;
    return 1U;
}
