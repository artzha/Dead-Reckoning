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
void synchronizeControllers(uint32_t I2C_1, Time *timer, uint8_t sender) {
    /* Sync with uC connected on I2C1 line */
    uint8_t millis_store[2];
    uint8_t seconds_store[4];
    uint8_t offset[5];

    /* Decide Behavior depending on Master Slave Status */
    if (sender) {
        /* PART 1 of Synchronization Procedure */

        /* Update millis store */
        i2c_transfer7(I2C_1, MPU_ADDR_SYNC, millis_store, 0, millis_store, 2);
        /* Update seconds store */
        i2c_transfer7(I2C_1, MPU_ADDR_SYNC, millis_store, 0, seconds_store, 4);

        /* Calculate Offset Value */
        uint16_t temp_millis    = millis_store[1]<<8|millis_store[0];
        uint32_t temp_seconds   = seconds_store[3]<<24|seconds_store[2]<<16|
                                seconds_store[1]<<8|seconds_store[0];
        
        int16_t offset_millis   = temp_millis - timer->millis;
        int32_t offset_seconds  = temp_seconds - timer->seconds;
        int32_t offset_total    = offset_millis + offset_seconds*1000;

        /*  Offset is stored as such
            [ 1 for positive/0 for negative, millis, millis, 1/0, seconds, seconds, seconds, seconds ]
        */
        offset[0] = offset_total >= 0 ? 1 : 0;
        offset[1] = (timer->offset>>24) & 0xFF;
        offset[2] = (timer->offset>>16) & 0xFF;
        offset[3] = (timer->offset>>8) & 0xFF;
        offset[4] = timer->offset & 0xFF;
        i2c_transfer7(I2C_1, MPU_ADDR_SYNC, offset, 5, offset, 0);

        /* PART 2 of Synchronization Procedure */

        uint8_t slave_time_arr[4];
        i2c_transfer7(I2C_1, MPU_ADDR_SYNC, slave_time_arr, 0, slave_time_arr, 4);
        int32_t delay = timer->millis + timer->seconds*1000;
        int32_t slave_time = bytesToTime(slave_time_arr);

        delay = (delay - slave_time)/2;

        /* Send newly calculated transimission time, disregarding sign */
        uint8_t delay_bytes[4];
        timeToBytes(delay, delay_bytes);
        i2c_transfer7(I2C_1, MPU_ADDR_SYNC, delay_bytes, 4, delay_bytes, 0);
    } else {
        /* PART 1 of Synchronization Procedure */

        /* Update current slave time */
        uint32_t slave_time = timer->millis + timer->seconds*1000;
        uint8_t time_store[4];
        timeToBytes(slave_time, time_store);

        millis_store[0] = *time_store;
        millis_store[1] = *(time_store+1);
        seconds_store[0] = *(time_store+2);
        seconds_store[1] = *(time_store+3);
        seconds_store[2] = *(time_store+4);
        seconds_store[3] = *(time_store+5);

        /* Send millis store */
        i2c_transfer7(I2C_1, MPU_ADDR_SYNC, millis_store, 2, millis_store, 0);
        /* Send seconds store */
        i2c_transfer7(I2C_1, MPU_ADDR_SYNC, seconds_store, 4, seconds_store, 0);

        /* Receive offset amount from master and calibrate */
        i2c_transfer7(I2C_1, MPU_ADDR_SYNC, offset, 0, offset, 5);

        int32_t amount = bytesToTime(offset+1);
        amount = (offset[0] == 0) ? amount*-1 : amount;
        /* Update time on slave with offset */
        timer->offset = -amount;
        updateTime(timer, timer->offset);

        /* PART 2 of Synchronization Procedure */

        /* Send current time in slave device */
        slave_time = timer->millis + timer->seconds*1000;
        uint8_t slave_time_bytes[4];
        timeToBytes(slave_time, slave_time_bytes);
        i2c_transfer7(I2C_1, MPU_ADDR_SYNC, slave_time_bytes, 4, slave_time_bytes, 0);

        /* Receive delay calculation from master and do final adjustment */
        i2c_transfer7(I2C_1, MPU_ADDR_SYNC, slave_time_bytes, 0, slave_time_bytes, 4);
        int32_t delay = bytesToTime(slave_time_bytes);
        timer->delay = -delay;
        updateTime(timer, timer->delay);
    }
}

void timeToBytes(uint32_t time, uint8_t*bytes) {
    uint8_t i = 0;
    *(bytes+3) = time & 0xFF;
    while(i < 3) {
        *(bytes+i) = time>>(8*(3-i))&0xFF;
        i++;
    }
}

uint32_t bytesToTime(uint8_t *bytes) {
    uint32_t time = bytes[0]<<24|bytes[1]<<16|bytes[2]<<8|bytes[3];
    return time;
}