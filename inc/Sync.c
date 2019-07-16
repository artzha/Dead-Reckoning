#include "Sync.h"

/*
    Updates timer struct in accordance to interrupt period t, 1 ms
*/
void updateTime(Time *timer, int32_t amount) {
    if (amount > 0) {
        int32_t diff = (amount + timer->millis) % 1000;
        /* Increment seconds counter */
        timer->seconds += (amount + timer->millis)/1000;
        /* Increment millisecond counter */
        timer->millis = diff;
    } else {
        /* Case 1: subtracting more than register value
            Case 2: subtracting less than regsiter value */
        uint16_t temp_millis = timer->millis;
        timer->millis = -(amount%1000) > temp_millis ?
            1000 + (amount%1000) + temp_millis: temp_millis + amount%1000;
        timer->seconds += -((-amount + temp_millis)/1000);
    }
}

/*
    rwBuffer requires a very specific data format
    @rwBuffer - {MILLIS_1, MILLIS_2,
        SECONDS_1, SECONDS_2, SECONDS_3, SECONDS_4}
    @sender - Master is indicated with 1 and slave is indicated with a zero
*/
void synchronizeControllers(uint32_t I2C_x, Time *timer, uint8_t sender) {
    /* Sync with uC connected on I2C1 line */
    uint8_t time_store[6];
    uint8_t offset[5];

    /* Decide Behavior depending on Master Slave Status */
    if (sender) {
        /* PART 1 of Synchronization Procedure */

        /* Updates time store millis|seconds*/
        i2c_transfer7(I2C_x, MPU_ADDR_SLAVE, time_store, 0, time_store, 6);

        /* Calculate Offset Value */
        uint16_t temp_millis    = time_store[0]<<8|time_store[1];
        uint32_t temp_seconds   = time_store[2]<<24|time_store[3]<<16|
                                time_store[4]<<8|time_store[5];

        int16_t offset_millis   = temp_millis - timer->millis;
        int32_t offset_seconds  = temp_seconds - timer->seconds;
        int32_t offset_total    = offset_millis + offset_seconds*1000;

        /*  Offset is stored as such
            [ 1 for positive/0 for negative, millis, millis, 1/0, seconds, seconds, seconds, seconds ]
        */
        offset[0] = offset_total >= 0 ? 1 : 0;
        offset[1] = (offset_total>>24) & 0xFF;
        offset[2] = (offset_total>>16) & 0xFF;
        offset[3] = (offset_total>>8) & 0xFF;
        offset[4] = offset_total & 0xFF;
        i2c_transfer7(I2C_x, MPU_ADDR_SLAVE, offset, 5, offset, 0);

        /* PART 2 of Synchronization Procedure */

        i2c_transfer7(I2C_x, MPU_ADDR_SLAVE, time_store, 0, time_store, 6);
        temp_millis    = time_store[0]<<8|time_store[1];
        temp_seconds   = time_store[2]<<24|time_store[3]<<16|
                                time_store[4]<<8|time_store[5];
        uint32_t slave_time = temp_millis + temp_seconds*1000.0;

        /* Calculate Delay using Master - Slave Time */
        int32_t delay = timer->millis + timer->seconds*1000.0;
        delay = (delay - slave_time)/2.0;
        timer->delay = delay;

        /* Send newly calculated transimission time, including sign */
        uint8_t delay_bytes[5];
        timeToBytes(delay, delay_bytes+1);
        delay_bytes[0] = delay >= 0 ? 1 : 0;
        i2c_transfer7(I2C_x, MPU_ADDR_SLAVE, delay_bytes, 5, delay_bytes, 0);
    }
}

void synchronizeOrientation(uint32_t I2C_1, uint32_t I2C_2, MPU_Init *mpu, Time *timer) {
    /* Store 4 bytes for pitch, roll, and yaw respectively */
    float average[3]; // stores average of pitch, roll, and yaw for current cycle
    float pitch[2], roll[2], yaw[2];
    uint8_t orientation_slave1[15];
    uint8_t orientation_slave2[15];
    /* Request pitch, roll, and yaw from slave */
    i2c_transfer7(I2C_1, MPU_ADDR_SLAVE, orientation_slave1, 0, orientation_slave1, 15);
    i2c_transfer7(I2C_2, MPU_ADDR_SLAVE, orientation_slave2, 0, orientation_slave2, 15);
    int i = 0;
    while(i < 3) {
        uint8_t sign_1 = orientation_slave1[5*i];
        float measurement_1 = orientation_slave1[(5*i)+1]<<24 | orientation_slave1[(5*i)+2]<<16 |
                                orientation_slave1[(5*i)+3]<<8 | orientation_slave1[(5*i)+4];

        uint8_t sign_2 = orientation_slave2[5*i];
        float measurement_2 = orientation_slave2[(5*i)+1]<<24 | orientation_slave2[(5*i)+2]<<16 |
                                orientation_slave2[(5*i)+3]<<8 | orientation_slave2[(5*i)+4];
        /* Normalize and convert measurement to float */
        measurement_1 /= 1000.0;
        measurement_1 = sign==0 ? measurement_1*(-1.0) : measurement_1;

        measurement_2 /= 1000.0;
        measurement_2 = sign==0 ? measurement_2*(-1.0) : measurement_2;
        switch(i) {
            case 0:
                pitch[0] = measurement_1;
                pitch[1] = measurement_2;
                average[i] = (pitch[0] + pitch[1] + mpu.pitch)/3.0;
                break;
            case 1:
                roll[0] = measurement_1;
                roll[1] = measurement_2;
                average[i] = (roll[0] + roll[1] + mpu.roll)/3.0;
                break;
            case 2:
                yaw[0] = measurement_1;
                yaw[1] = measurement_2;
                average[i] = (yaw[0] + yaw[1] + mpu.yaw)/3.0;
                break;
            default:
                break;
        }
        i++;
    }

    /* Default To Byzantine Generals Algorithm */
    timer.totalRuns++;
    // Calculate Standard Deviations of Runs To Use For determining correctness
    // outer loop for pitch, roll, yaw
    i = 0;
    while(i < 3) {
        float stdDev = 0;
        switch(i) {
            case 0:
                // save pitch vars here and calculate stdDev
            case 1:

            case 2:

            default:
                break;
        }

        // inner loop for determining each uC's correctness
        int j = 0;
        while (j < 3) {
            // Compare and store each sensors correctness in timer.correct
            // Truth values in timer.correct should be ANDED together
            j++;
        }
        i++;
    }

}

void timeToBytes(uint32_t time, uint8_t *bytes) {
    int i = 0;
    *(bytes+3) = time & 0xFF;
    while(i < 3) {
        *(bytes+i) = time>>(8*(3-i))&0xFF;
        i++;
    }
}

uint32_t bytesToTime(volatile uint8_t *bytes) {
    return bytes[0]<<24|bytes[1]<<16|bytes[2]<<8|bytes[3];
}
