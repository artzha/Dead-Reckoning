#include "Sync.h"

/* 
    Updates timer struct in accordance to interrupt period t, 1 ms
*/
void update_time(Time *timer) {
    if (timer->millis == 999) {
        /* Reset millisecond counter */
        timer->millis = 0;
        /* Increment seconds counter */
        timer->seconds++;
    } else {
        timer->millis++;
    }
}

/*
    rwBuffer requires a very specific data format
    @rwBuffer - {MILLIS_1, MILLIS_2, 
        SECONDS_1, SECONDS_2, SECONDS_3, SECONDS_4}
    @sender - Master is indicated with 1 and slave is indicated with a zero
*/
void synchronize_controllers(uint32_t I2C_1, uint32_t I2C_2, Time *timer, uint8_t sender, uint8_t *rwBuffer) {
    /* Sync with uC connected on I2C1 line */
    uint8_t millis_store[2];
    uint8_t seconds_store[4];

    /* Decide Behavior depending on Master Slave Status */
    if (sender) {
        /* Update millis store */
        i2c_transfer7(I2C_1, MPU_ADDR, rwBuffer, 2, millis_store, 2);
        /* Update seconds store */
        i2c_transfer7(I2C_1, MPU_ADDR, rwBuffer+2, 4, seconds_store, 4);

        /* Calculate Offset Value */
        uint16_t temp_millis    = millis_store[1]<<8|millis_store[0];
        uint32_t temp_seconds   = seconds_store[3]<<24|seconds_store[2]<<16|
                                seconds_store[1]<<8|seconds_store[0];
        
        timer->offset_millis    = temp_millis - timer->millis;
        timer->offset_seconds   = temp_seconds - timer->seconds;
    } else {
        /* Send millis store */
        i2c_transfer7(I2C_1, MPU_ADDR, rwBuffer, 0, rwBuffer, 2);
        /* Send seconds store */
        i2c_transfer7(I2C_1, MPU_ADDR, rwBuffer, 0, rwBuffer+2, 4);
    }

    /* Calculate adjustment for communication delay */

}