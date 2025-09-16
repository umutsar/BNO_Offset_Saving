#ifndef BNO055_H
#define BNO055_H

#include "stm32f1xx_hal.h"   // Kartına göre değiştir

// I2C adresi: ADDR GND -> 0x28
#define BNO055_I2C_ADDR   (0x28 << 1)

// Kimlik ve temel registerlar
#define BNO055_CHIP_ID_ADDR       0x00
#define BNO055_CHIP_ID_VAL        0xA0
#define BNO055_PAGE_ID_ADDR       0x07
#define BNO055_OPR_MODE_ADDR      0x3D
#define BNO055_PWR_MODE_ADDR      0x3E
#define BNO055_SYS_TRIGGER_ADDR   0x3F
#define BNO055_UNIT_SEL_ADDR      0x3B
#define BNO055_SYS_STATUS_ADDR    0x39
#define BNO055_SYS_ERR_ADDR       0x3A
#define BNO055_CALIB_STAT_ADDR    0x35
#define BNO055_ST_RESULT_ADDR     0x36

// Modlar
#define BNO055_OPMODE_CONFIG      0x00
#define BNO055_OPMODE_NDOF        0x0C
#define BNO055_PWRMODE_NORMAL     0x00

// Veri registerları
#define BNO055_EULER_H_LSB        0x1A   // 6 bayt: H, R, P (LSB..MSB)
#define BNO055_QUAT_W_LSB         0x20   // 8 bayt: W, X, Y, Z (LSB..MSB)
#define BNO055_LIA_X_LSB          0x28   // Linear Accel (m/s²)
#define BNO055_GRV_X_LSB          0x2E   // Gravity (m/s²)

// Kalibrasyon offset sayfası (PAGE 0)
#define BNO055_ACC_OFFSET_LSB     0x55   // Toplam 22 bayt (ACC,GYR,MAG offset + radius)

// Ölçekler
#define BNO055_EULER_SCALE        (16.0f)      // 16 LSB/deg
#define BNO055_QUAT_SCALE         (16384.0f)   // 1/16384
#define BNO055_ACC_SCALE          (100.0f)     // 100 LSB/(m/s^2)
#define BNO055_GYRO_SCALE_DPS     (16.0f)      // 16 LSB/dps

typedef struct {
    int16_t acc_offset_x;
    int16_t acc_offset_y;
    int16_t acc_offset_z;
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    int16_t mag_offset_z;
    int16_t gyr_offset_x;
    int16_t gyr_offset_y;
    int16_t gyr_offset_z;
    int16_t acc_radius;
    int16_t mag_radius;
} BNO055_CalibProfile_t;

// API
uint8_t BNO055_Init(I2C_HandleTypeDef *hi2c);
uint8_t BNO055_SetOpMode(I2C_HandleTypeDef *hi2c, uint8_t mode);
uint8_t BNO055_SetUnits_Default(I2C_HandleTypeDef *hi2c); // °, °/s, m/s², °C
uint8_t BNO055_ReadEuler(I2C_HandleTypeDef *hi2c, float *heading, float *roll, float *pitch);
uint8_t BNO055_ReadQuaternion(I2C_HandleTypeDef *hi2c, float *w, float *x, float *y, float *z);
uint8_t BNO055_ReadLinearAccel(I2C_HandleTypeDef *hi2c, float *ax, float *ay, float *az);
uint8_t BNO055_ReadGravity(I2C_HandleTypeDef *hi2c, float *gx, float *gy, float *gz);

uint8_t BNO055_GetCalibStatus(I2C_HandleTypeDef *hi2c, uint8_t *sys, uint8_t *gyro, uint8_t *acc, uint8_t *mag);
uint8_t BNO055_WaitFullyCalibrated(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms);
uint8_t BNO055_ReadCalibProfile(I2C_HandleTypeDef *hi2c, BNO055_CalibProfile_t *prof);
uint8_t BNO055_WriteCalibProfile(I2C_HandleTypeDef *hi2c, const BNO055_CalibProfile_t *prof);

#endif
