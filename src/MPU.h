#ifndef MPU_h
#define MPU_h

#include <stdint.h>
#include <math.h>
#include <libopencm3/stm32/i2c.h>

/* MPU Register Definitions */
#define MPU_ADDR			    0x68
#define MPU_ADDR_PW_MGT_CFG	    0x6B
#define ACC_ADDR_CFG 		    0x1B
#define GYRO_ADDR_CFG 		    0x1C
#define MAG_ADDR                0x0C

/* Interrupts */
#define MPU_INT_BYPASS_CFG      0x37
#define MPU_INT_ENABLE          0x38                

/* Magnetometer Register Definitions */
#define MAG_FIFO_ENABLE         0x23
#define MAG_I2C_SLV0_CTRL       0x27
#define MAG_I2C_SLV0_REG        0x26
#define MAG_CNTL                0x0A
#define MAG_ASAX                0x10
#define MAG_ASAY                0x11
#define MAG_ASAZ                0x12
#define MAG_ST1                 0x02
#define MAG_HXL                 0x03

#define ACC_OUT_X_L			    0x3B
#define ACC_OUT_X_H			    0x3C

/* Global Constants for AHRS */
#define PI                      3.141592653
#define GyroMeasError           PI * (4.0f / 180.0f)
#define GyroMeasDrift           PI * (0.0f  / 180.0f)

typedef struct {
    float *magCal, *gyroCal;
    float magCalibration[3];        // x/y/z gyro calibration data stored here
    float acc[3], gyro[3], mag[3];
    float pitch, yaw, roll;
                // a12, a22, a31, a32, a33; rotation matrix coefficients for Euler angles and gravity components
    float deltat;
    float q[4];
} MPU_Init;

void mpuSetup(uint32_t I2C, MPU_Init *mpu);
void initMagnetometer(uint32_t I2C, float* magCalibration);
void readAccelerometer(uint32_t I2C, float *acc);
void readGyroscope(uint32_t I2C, float *gyro);
void readMagnetometer(uint32_t I2C, float *mag, float* magCalibration);
void madgwickQuaternionRefresh(float *q, MPU_Init *mpu, float* acc, float* gyro, float* mag);
void quarternionToEulerAngle(float *q, float *pitch, float *yaw, float *roll);

#endif