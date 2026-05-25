#ifndef MPU6050_RAW_H
#define MPU6050_RAW_H

#include <stdint.h>

typedef struct {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    float accel_mps2[3];
    float gyro_rads[3];
    float temperature_c;
} mpu6050_sample_t;

void mpu6050_raw_bus_init(void);
uint8_t mpu6050_raw_init(void);
uint8_t mpu6050_raw_read_whoami(void);
uint8_t mpu6050_raw_read(mpu6050_sample_t *sample);

#endif
