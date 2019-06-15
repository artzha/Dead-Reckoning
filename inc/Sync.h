#ifndef SYNC_h
#define SYNC_h

#include <libopencm3/stm32/i2c.h>

#define MPU_ADDR_SLAVE          0x32
#define MPU_ADDR_MASTER         0x32
#define REQ_TIME_SPLIT          (uint8_t) 0x01
#define REQ_TIME_NORM           (uint8_t) 0x02
#define 

typedef struct {
    /* Set as volatile since uC is 16 bit and this is modified in interrupts*/
    volatile uint16_t millis;
    volatile uint32_t seconds;
    volatile int32_t offset; // stored in milliseconds
    volatile int32_t delay;
} Time;

void updateTime(Time *timer, int32_t amount);
void synchronizeControllers(uint32_t I2C_1, Time *timer, uint8_t sender);
void timeToBytes(uint32_t time, uint8_t*bytes);
uint32_t bytesToTime(uint8_t *bytes);

#endif