#include "Sync.h"

/* 
    Updates timer struct in accordance to interrupt period t, 1 ms
*/
void updateTime(Time *timer, int32_t amount) {
    int32_t diff = (amount + timer->millis) % 1000;
    /* Increment millisecond counter */
    timer->millis = diff;
    /* Increment seconds counter */
    timer->seconds += (amount + timer->millis)/1000;
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

void synchronizeOrientation(uint32_t I2C_x, MPU_Init *mpu, Time *timer) {
    /* Store 4 bytes for pitch, roll, and yaw respectively */
    int32_t pitch, roll, yaw;
    int8_t orientation[12];
    /* Request pitch, roll, and yaw from slave */
    i2c_transfer7(I2C_x, MPU_ADDR_SLAVE, orientation, 0, orientation, 12);
    uint8_t i = 0;
    while(i++ < 3) {
        int32_t measurement = orientation[4*i]<<24 | orientation[4*i+1]<<16 |
                                orientation[4*i+2]<<8 | orientation[4*i+3];
        switch(i) {
            case 0:
                pitch = measurement;
                break;
            case 1: 
                roll = measurement;
                break;
            case 2:
                yaw = measurement;
                break;
            default:
                break;
        }
    }
    
    /* Compare slave calculations to self */
}

void timeToBytes(uint32_t time, uint8_t *bytes) {
    uint8_t i = 0;
    *(bytes+3) = time & 0xFF;
    while(i < 3) {
        *(bytes+i) = time>>(8*(3-i))&0xFF;
        i++;
    }
}

uint32_t bytesToTime(volatile uint8_t *bytes) {
    uint32_t time = bytes[0]<<24|bytes[1]<<16|bytes[2]<<8|bytes[3];
    return time;
}