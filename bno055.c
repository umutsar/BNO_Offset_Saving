#include "bno055.h"
#include <string.h>

#define BNO_ADDR BNO055_I2C_ADDR

static uint8_t mem_read(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len) {
    return (HAL_I2C_Mem_Read(hi2c, BNO_ADDR, reg, 1, buf, len, HAL_MAX_DELAY) == HAL_OK);
}
static uint8_t mem_write(I2C_HandleTypeDef *hi2c, uint8_t reg, const uint8_t *buf, uint16_t len) {
    return (HAL_I2C_Mem_Write(hi2c, BNO_ADDR, reg, 1, (uint8_t*)buf, len, HAL_MAX_DELAY) == HAL_OK);
}
static uint8_t write_u8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val) {
    return mem_write(hi2c, reg, &val, 1);
}
static uint8_t read_u8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val) {
    return mem_read(hi2c, reg, val, 1);
}

uint8_t BNO055_SetOpMode(I2C_HandleTypeDef *hi2c, uint8_t mode) {
    if (!write_u8(hi2c, BNO055_OPR_MODE_ADDR, mode)) return 0;
    HAL_Delay(30);
    return 1;
}

uint8_t BNO055_SetUnits_Default(I2C_HandleTypeDef *hi2c) {
    return write_u8(hi2c, BNO055_UNIT_SEL_ADDR, 0x00);
}

uint8_t BNO055_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t id = 0;

    if (!read_u8(hi2c, BNO055_CHIP_ID_ADDR, &id)) return 0;
    if (id != BNO055_CHIP_ID_VAL) return 0;

    if (!BNO055_SetOpMode(hi2c, BNO055_OPMODE_CONFIG)) return 0;

    if (!write_u8(hi2c, BNO055_PWR_MODE_ADDR, BNO055_PWRMODE_NORMAL)) return 0;
    HAL_Delay(10);

    if (!BNO055_SetUnits_Default(hi2c)) return 0;

    if (!write_u8(hi2c, BNO055_SYS_TRIGGER_ADDR, 0x20)) return 0; // Reset
    HAL_Delay(650);

    if (!BNO055_SetOpMode(hi2c, BNO055_OPMODE_CONFIG)) return 0;

    // Harici kristal her zaman aktif
    if (!write_u8(hi2c, BNO055_SYS_TRIGGER_ADDR, 0x80)) return 0;
    HAL_Delay(10);

    if (!BNO055_SetOpMode(hi2c, BNO055_OPMODE_NDOF)) return 0;
    HAL_Delay(30);

    return 1;
}

uint8_t BNO055_ReadEuler(I2C_HandleTypeDef *hi2c, float *heading, float *roll, float *pitch) {
    uint8_t buf[6];
    if (!mem_read(hi2c, BNO055_EULER_H_LSB, buf, 6)) return 0;
    int16_t h = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t r = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t p = (int16_t)((buf[5] << 8) | buf[4]);
    if (heading) *heading = (float)h / BNO055_EULER_SCALE;
    if (roll)    *roll    = (float)r / BNO055_EULER_SCALE;
    if (pitch)   *pitch   = (float)p / BNO055_EULER_SCALE;
    return 1;
}

uint8_t BNO055_ReadQuaternion(I2C_HandleTypeDef *hi2c, float *w, float *x, float *y, float *z) {
    uint8_t buf[8];
    if (!mem_read(hi2c, BNO055_QUAT_W_LSB, buf, 8)) return 0;
    int16_t W = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t X = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t Y = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t Z = (int16_t)((buf[7] << 8) | buf[6]);
    if (w) *w = (float)W / BNO055_QUAT_SCALE;
    if (x) *x = (float)X / BNO055_QUAT_SCALE;
    if (y) *y = (float)Y / BNO055_QUAT_SCALE;
    if (z) *z = (float)Z / BNO055_QUAT_SCALE;
    return 1;
}

uint8_t BNO055_ReadLinearAccel(I2C_HandleTypeDef *hi2c, float *ax, float *ay, float *az) {
    uint8_t buf[6];
    if (!mem_read(hi2c, BNO055_LIA_X_LSB, buf, 6)) return 0;
    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);
    if (ax) *ax = (float)x / BNO055_ACC_SCALE;
    if (ay) *ay = (float)y / BNO055_ACC_SCALE;
    if (az) *az = (float)z / BNO055_ACC_SCALE;
    return 1;
}

uint8_t BNO055_ReadGravity(I2C_HandleTypeDef *hi2c, float *gx, float *gy, float *gz) {
    uint8_t buf[6];
    if (!mem_read(hi2c, BNO055_GRV_X_LSB, buf, 6)) return 0;
    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);
    if (gx) *gx = (float)x / BNO055_ACC_SCALE;
    if (gy) *gy = (float)y / BNO055_ACC_SCALE;
    if (gz) *gz = (float)z / BNO055_ACC_SCALE;
    return 1;
}

uint8_t BNO055_GetCalibStatus(I2C_HandleTypeDef *hi2c, uint8_t *sys, uint8_t *gyro, uint8_t *acc, uint8_t *mag) {
    uint8_t v=0;
    if (!read_u8(hi2c, BNO055_CALIB_STAT_ADDR, &v)) return 0;
    if (sys)  *sys  = (v >> 6) & 0x03;
    if (gyro) *gyro = (v >> 4) & 0x03;
    if (acc)  *acc  = (v >> 2) & 0x03;
    if (mag)  *mag  = (v >> 0) & 0x03;
    return 1;
}

uint8_t BNO055_WaitFullyCalibrated(I2C_HandleTypeDef *hi2c, uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        uint8_t s,g,a,m;
        if (!BNO055_GetCalibStatus(hi2c, &s, &g, &a, &m)) return 0;
        if (s==3 && g==3 && a==3 && m==3) return 1;
        HAL_Delay(100);
    }
    return 0; // timeout
}

uint8_t BNO055_ReadCalibProfile(I2C_HandleTypeDef *hi2c, BNO055_CalibProfile_t *prof) {
    if (!BNO055_SetOpMode(hi2c, BNO055_OPMODE_CONFIG)) return 0;
    uint8_t raw[22];
    if (!mem_read(hi2c, BNO055_ACC_OFFSET_LSB, raw, 22)) return 0;
    int16_t *p = (int16_t*)raw;
    prof->acc_offset_x = p[0];
    prof->acc_offset_y = p[1];
    prof->acc_offset_z = p[2];
    prof->mag_offset_x = p[3];
    prof->mag_offset_y = p[4];
    prof->mag_offset_z = p[5];
    prof->gyr_offset_x = p[6];
    prof->gyr_offset_y = p[7];
    prof->gyr_offset_z = p[8];
    prof->acc_radius   = p[9];
    prof->mag_radius   = p[10];
    return 1;
}

uint8_t BNO055_WriteCalibProfile(I2C_HandleTypeDef *hi2c, const BNO055_CalibProfile_t *prof) {
    if (!BNO055_SetOpMode(hi2c, BNO055_OPMODE_CONFIG)) return 0;
    uint8_t raw[22];
    int16_t *p = (int16_t*)raw;
    p[0]=prof->acc_offset_x; p[1]=prof->acc_offset_y; p[2]=prof->acc_offset_z;
    p[3]=prof->mag_offset_x; p[4]=prof->mag_offset_y; p[5]=prof->mag_offset_z;
    p[6]=prof->gyr_offset_x; p[7]=prof->gyr_offset_y; p[8]=prof->gyr_offset_z;
    p[9]=prof->acc_radius;   p[10]=prof->mag_radius;
    if (!mem_write(hi2c, BNO055_ACC_OFFSET_LSB, raw, 22)) return 0;
    if (!BNO055_SetOpMode(hi2c, BNO055_OPMODE_NDOF)) return 0;
    HAL_Delay(30);
    return 1;
}
