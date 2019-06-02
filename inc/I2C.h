#include "stm32f1xx.h"
#include "stm32f103xb.h"

/*********************** defines                    *************************/
#define MPU_ADDRESS                     (uint8_t)0x68
#define BLUE_PILL                       0x0410
#define MPU_DELAY   150000UL
#define I2C_IT_ERR                      ((uint16_t)0x0100)

void init_gpio_pins();
void i2c_start(uint8_t num_bytes);
void i2c_send_7bit_address(uint32_t slave_address);
void i2c_stop();
uint8_t i2c_receive_data();
void i2c_send_data(uint8_t data);

/*  Function:       I2C_Low_Level_Init
    Description:    Configure i2c settings for blue pill
    Parameters:     void
    Returns:        void
*/
void I2C_Low_Level_Init(int ClockSpeed, int OwnAddress);

/* 
    Handles everything needed to write to address
*/
void I2C_Write(const uint8_t *buf, uint32_t nbyte, uint8_t SlaveAddress);

/* 
    Handles everything needed to write to address
*/
void I2C_Read(uint8_t *buf, uint32_t nbyte, uint8_t SlaveAddress);

/*  Function:       init_clock()
    Description:    configure SysClock to run at 72MHz
    Parameters:     void
    Returns:        void 
*/
void init_clock(void);

void init_hardware();

void init_MPU9250();