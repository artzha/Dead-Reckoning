#ifndef MPU_h
#define MPU_h
#include <stdint.h>
#include <libopencm3/stm32/i2c.h>

/* MPU Register Definitions */
#define MPU_ADDR			0x68
#define MPU_ADDR_PW_MGT_CFG	0x6B
#define ACC_ADDR_CFG 		0x1B
#define GYRO_ADDR_CFG 		0x1C

#define ACC_OUT_X_L			0x3B
#define ACC_OUT_X_H			0x3C

class MPU {
    public:
        MPU(uint32_t I2C);
        void readAccelerometer(double *acc);

    private:
        void mpuSetup();
}

#endif