#include "MPU.h"

void mpuSetup(uint32_t I2C, double* magCalibration) {
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
	initMagnetometer(I2C, magCalibration);
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
	int i = 0;
	while(i < 3) {
		mag[i] = mag[i]*magCalibration[i];
		i++;
	}

}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2) {  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }

    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
  
// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);


// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}