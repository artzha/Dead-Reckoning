#include "MPU.h"

void mpuSetup(uint32_t I2C) {
	uint8_t data[2] = { MPU_ADDR_PW_MGT_CFG, 0 };
	i2c_transfer7(I2C, MPU_ADDR, data, 2, data, 0);

	/* Configure  Accelerometer */
	data[0] = ACC_ADDR_CFG;
	data[1] = 0b00011000;
	i2c_transfer7(I2C, MPU_ADDR, data, 2, data, 0);

	/* Configure Gyroscope */
	data[0] = GYRO_ADDR_CFG;
	data[1] = 0b00011000;
	i2c_transfer7(I2C, MPU_ADDR, data, 2, data, 0);

	/* Configure Magnetometer */

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master
	data[0] = MPU_INT_BYPASS_CFG;
	data[1] = 0x22;
	i2c_transfer7(I2C, MPU_ADDR, data, 2, data, 0);
	data[0] = MPU_INT_ENABLE;
	data[1] = 0x01;
	i2c_transfer7(I2C, MPU_ADDR, data, 2, data, 0);  // Enable data ready (bit 0) interrupt

	/* Enable Magnetometer */
	initMagnetomer(I2C);
}

void initMagnetomer(uint32_t I2C) {
	uint8_t data[2] = { MAG_CNTL, 0x00};
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	i2c_transfer7(I2C, MAG_ADDR, data, 2, data, 0); // Power down magnetometer
	data[1] = 0x0F;
	i2c_transfer7(I2C, MAG_ADDR, data, 2, data, 0); // Enter Fuse ROM access mode

	/* Calibrate using Factory Default */
	data[0] = MAG_ASAX;
	i2c_transfer7(I2C, MAG_ADDR, data, 1, rawData, 1);
	data[0] = MAG_ASAY;
	i2c_transfer7(I2C, MAG_ADDR, data, 1, rawData+1, 1);
	data[0] = MAG_ASAZ;
	i2c_transfer7(I2C, MAG_ADDR, data, 1, rawData+2, 1);

	rawData[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	rawData[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
	rawData[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;

	data[0] = MAG_CNTL;
	data[1] = 0x00;
	i2c_transfer7(I2C, MAG_ADDR, data, 1, data, 0); // Power down magnetometer  

	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	data[0] = MAG_CNTL;
	data[1] = 0b10110;
	i2c_transfer7(I2C, MAG_ADDR, data, 2, data, 0);
}

void readAccelerometer(uint32_t I2C,double* acc) {
	uint8_t raw[6];
	uint8_t addr[6] = { 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40 };
	int i = 0;
	while(i < 6) {
		i2c_transfer7(I2C, MPU_ADDR, addr+i, 1, raw+i, 1);
		i++;
	}

	acc[0] = (int16_t)((int16_t)raw[0]<<8|raw[1]);
	acc[1] = (int16_t)((int16_t)raw[2]<<8|raw[3]);
	acc[2] = (int16_t)((int16_t)raw[4]<<8|raw[5]);

	acc[0] = acc[0]/2048.0;
	acc[1] = acc[1]/2048.0;
	acc[2] = acc[2]/2048.0;
}

void readGyroscope(uint32_t I2C, double *gyro) {
	uint8_t raw[6];
	uint8_t addr[6] = { 0x43, 0x44, 0x45, 0x46, 0x47, 0x48 };
	int i = 0;
	while(i < 6) {
		i2c_transfer7(I2C, MPU_ADDR, addr+i, 1, raw+i, 1);
		i++;
	}

	gyro[0] = (int16_t)((int16_t)raw[0]<<8|raw[1]);
	gyro[1] = (int16_t)((int16_t)raw[2]<<8|raw[3]);
	gyro[2] = (int16_t)((int16_t)raw[4]<<8|raw[5]);

	gyro[0] = gyro[0]/16.4;
	gyro[1] = gyro[1]/16.4;
	gyro[2] = gyro[2]/16.4;
}

void readMagnetometer(uint32_t I2C, double *mag) {
	uint8_t data[2] = { MAG_ST1, 0 };
	uint8_t status[2];
	i2c_transfer7(I2C, MAG_ADDR, data, 1, status, 1);
	
	uint8_t mag_data[7];
	uint8_t mag_register = MAG_HXL;
	/* Read In All Magnetometer Values and Overflow Status */
	if (status[0] & 0x01) {
		int i = 0;
		while(++i < 7) {
			i2c_transfer7(I2C, MAG_ADDR, mag_register + i, 1, mag_data + i, 1);
		}
	}

	/* If Data Was Not Overflowed, Read in New Data */
	if(!(mag_data[6] & 0x08)) {
		mag[0] = (int16_t)((int16_t)mag_data[1]<<8|mag_data[0]);
		mag[1] = (int16_t)((int16_t)mag_data[3]<<8|mag_data[2]);
		mag[2] = (int16_t)((int16_t)mag_data[5]<<8|mag_data[4]);
	}
	int i = 0;
	while(++i < 3) {
		mag[i] = 0.6/mag[i];
	}

}