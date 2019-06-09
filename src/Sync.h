#ifndef SYNC_h
#define SYNC_h

#include <libopencm3/stm32/i2c.h>

#define MPU_ADDR            0x32

typedef struct {
    /* Set as volatile since uC is 16 bit and this is modified in interrupts*/
    volatile uint16_t millis;
    volatile uint32_t seconds;
    volatile uint16_t offset_millis;
    volatile uint32_t offset_seconds;
} Time;

void update_time(Time *timer);
void synchronize_controllers(uint32_t I2C_1, uint32_t I2C_2, Time *timer, uint8_t sender, uint8_t *rwBuffer);

void receiveEventAsSlave();
void sendEventAsSlave();

#endif