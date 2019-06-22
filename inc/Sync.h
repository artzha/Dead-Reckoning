#ifndef SYNC_h
#define SYNC_h

#include <libopencm3/stm32/i2c.h>

#define MPU_ADDR_SLAVE          0x32
#define ADJ_OFFSET              (uint8_t) 0x01
#define ADJ_DELAY               (uint8_t) 0x02

typedef struct {
    /* Set as volatile since uC is 16 bit and this is modified in interrupts*/
    volatile uint16_t millis;
    volatile uint32_t seconds;
    volatile uint8_t offset[5]; // stored in milliseconds (raw data)
    volatile int32_t delay; // converted to time with signed values
    volatile int8_t mode; // config mode is 1, orientation mode is 2, initially set to 0
} Time;

void updateTime(Time *timer, int32_t amount);
void synchronizeControllers(uint32_t I2C_1, Time *timer, uint8_t sender);
void timeToBytes(uint32_t time, uint8_t*bytes);
uint32_t bytesToTime(volatile uint8_t *bytes);

#endif