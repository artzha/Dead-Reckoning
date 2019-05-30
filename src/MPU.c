#include "MPU.h"

void mpuSetup(uint32_t I2C, MPU_Init *mpu) {
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
	initMagnetometer(I2C, mpu->magCalibration);

	/* Set Filter Variables */
	mpu->deltat = 0.5; // Sets Update Rate For Sensors and Orientation Calculation to 0.5 second intervals
}

void initMagnetometer(uint32_t I2C, double* magCalibration) {
	uint8_t data[2] = { MAG_CNTL, 0x00};
	uint8_t rawCalibration[3];
	i2c_transfer7(I2C, MAG_ADDR, data, 2, data, 0); // Power down magnetometer
	data[1] = 0x0F;
	i2c_transfer7(I2C, MAG_ADDR, data, 2, data, 0); // Enter Fuse ROM access mode

	/* Calibrate using Factory Default */
	data[0] = MAG_ASAX;
	i2c_transfer7(I2C, MAG_ADDR, data, 1, rawCalibration, 1);
	data[0] = MAG_ASAY;
	i2c_transfer7(I2C, MAG_ADDR, data, 1, rawCalibration+1, 1);
	data[0] = MAG_ASAZ;
	i2c_transfer7(I2C, MAG_ADDR, data, 1, rawCalibration+2, 1);

	magCalibration[0] = (rawCalibration[0] - 128)/256.0 + 1.0;   // Return x-axis sensitivity adjustment values, etc.
	magCalibration[1] = (rawCalibration[1] - 128)/256.0 + 1.0;  
	magCalibration[2] = (rawCalibration[2] - 128)/256.0 + 1.0;

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

void readMagnetometer(uint32_t I2C, double *mag, double* magCalibration) {
	uint8_t data[2] = { MAG_ST1, 0 };
	uint8_t status[2];
	i2c_transfer7(I2C, MAG_ADDR, data, 1, status, 1);
	
	uint8_t mag_data[7];
	uint8_t mag_register[7];
	mag_register[0] = MAG_HXL;
	/* Read In All Magnetometer Values and Overflow Status */
	if (status[0] & 0x01) {
		int i = 0;
		while(i < 7) {
			mag_register[i] = mag_register[0] + i;
			i2c_transfer7(I2C, MAG_ADDR, mag_register+i, 1, mag_data + i, 1);
			i++;
		}
	}

	/* If Data Was Not Overflowed, Read in New Data */
	if(!(mag_data[6] & 0x08)) {
		mag[0] = (int16_t)((int16_t)mag_data[1]<<8|mag_data[0]);
		mag[1] = (int16_t)((int16_t)mag_data[3]<<8|mag_data[2]);
		mag[2] = (int16_t)((int16_t)mag_data[5]<<8|mag_data[4]);
	}

	double magOffset[3] = { -10.6523, 50.0391, 149.3613 };
	int i = 0;
	while(i < 3) {
		mag[i] = mag[i]*magCalibration[i] - magOffset[i];
		i++;
	}

}

void MadgwickQuarternionUpdate(float *q, MPU_Init *mpu, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrtf(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;
	
	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrtf(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * mpu->deltat;
	q2 += qDot2 * mpu->deltat;
	q3 += qDot3 * mpu->deltat;
	q4 += qDot4 * mpu->deltat;
	norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}