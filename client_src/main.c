#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include "../inc/MPU.h"
#include "../inc/Sync.h"

/* I2C interrupt readings */

volatile float update = 0;
volatile uint8_t reading;
Time timer = {0, 0, 0, 0}; // initialize timer values

static void clock_setup(void)
{
	rcc_clock_setup_in_hse_16mhz_out_72mhz();
}

static void gpio_setup(void)
{
	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Set GPIO6/7 (in GPIO port B) to 'open drain' for the LEDs. */
	// gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
	//               GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
	// gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
	//               GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
}

static void timer_setup(void)
{
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM3);
	/* Set timer start value. */
	TIM_CNT(TIM2) = 1;
	TIM_CNT(TIM3) = 1;

	/* Set timer prescaler. 72MHz/1440 => 50000 counts per second. */
	TIM_PSC(TIM2) = 1440;
	/* Set timer prescaler to 72MHz/72 => 1000000 counts per second */
	TIM_PSC(TIM3) = 72;

	/* End timer value. If this is reached an interrupt is generated. */
	TIM_ARR(TIM2) = 25000;
	/* End timer 1000000Hz/1000 -> 1 millisecond */
	TIM_ARR(TIM3) = 1000;

	/* Update interrupt enable. */
	TIM_DIER(TIM2) |= TIM_DIER_UIE;
	TIM_DIER(TIM3) |= TIM_DIER_UIE;

	/* Start timer. */
	TIM_CR1(TIM2) |= TIM_CR1_CEN;
	TIM_CR1(TIM3) |= TIM_CR1_CEN;
}

static void nvic_setup(void)
{
	/* Without this the timer interrupt routine will never be called. */
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_set_priority(NVIC_TIM2_IRQ, 1);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_set_priority(NVIC_TIM3_IRQ, 1);

	/* Without this the I2C1 interrupt routine will never be called. */
	nvic_enable_irq(NVIC_I2C1_EV_IRQ);
}

static void i2c_setup(void)
{
	/* Enable clocks for I2C2 and AFIO. */
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_AFIO);

	/* Set alternate functions for the SCL and SDA pins of I2C2. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		      GPIO_I2C1_SCL | GPIO_I2C1_SDA);

	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(I2C1);

	/* APB1 is running at 36MHz. */
	i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);

	/* 400KHz - I2C Fast Mode */
	i2c_set_fast_mode(I2C1);

	/*
	 * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
	 * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
	 * Datasheet suggests 0x1e.
	 */
	i2c_set_ccr(I2C1, 0x1e);

	/*
	 * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
	 * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
	 * Incremented by 1 -> 11.
	 */
	i2c_set_trise(I2C1, 0x0b);

	/*
	 * This is our slave address - needed only if we want to receive from
	 * other masters.
	 */
	i2c_set_own_7bit_slave_address(I2C1, 0x32);

	/* 
	 * This allows master to interrupt slave at any time to compare 
	 * calculation and synchronize time data
	 */
	i2c_enable_interrupt(I2C1, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(I2C1);

	// slave needs to acknowledge on receiving bytes
	// set it after enabling Peripheral i.e. PE = 1
	i2c_enable_ack(I2C1);
}

void i2c1_ev_isr(void)
{
   uint32_t sr1, sr2;

   sr1 = I2C_SR1(I2C1);

   /* Sync with uC connected on I2C1 line */
    uint8_t millis_store[2];
    uint8_t seconds_store[4];
    uint8_t offset[5];

	/* Update current slave time */
	uint32_t slave_time = timer.millis + timer.seconds*1000;
	uint8_t time_store[4];
	timeToBytes(slave_time, time_store);

   // Address matched (Slave)
   	if (sr1 & I2C_SR1_ADDR)
    {
        reading = 0;

        //Clear the ADDR sequence by reading SR2.
        sr2 = I2C_SR2(I2C1);
        (void) sr2;
    }
	// Receive buffer not empty
	else if (sr1 & I2C_SR1_RxNE)
	{
        //ignore more than 3 bytes reading
        if (reading > 3)
          return;

        /* Receive offset amount from master and calibrate */
        offset[reading++] = i2c_get_data(I2C1);
        reading++;
    }
   	// Transmit buffer empty & Data byte transfer not finished
   	else if ((sr1 & I2C_SR1_TxE) && !(sr1 & I2C_SR1_BTF))
    {
		/* Transmit time store */
		i2c_send_data(I2C1, millis_store);
		i2c_send_data(I2C1, seconds_store);
    }
	// done by master by sending STOP
	//this event happens when slave is in Recv mode at the end of communication
   	else if (sr1 & I2C_SR1_STOPF)
    {
        i2c_peripheral_enable(I2C1);

        /* Slave should calibrate it's time register after master reception */
		int32_t amount = bytesToTime(offset+1);
        amount = (offset[0] == 0) ? amount*-1 : amount;
        /* Update time on slave with offset */
        timer.offset = -amount;
        updateTime(&timer, timer.offset);
    }
 	//this event happens when slave is in transmit mode at the end of communication
   	else if (sr1 & I2C_SR1_AF)
    {


        //(void) I2C_SR1(I2C1);
        I2C_SR1(I2C1) &= ~(I2C_SR1_AF);
    }
}

void tim2_isr(void)
{
	/* Update Necessary Variables Here */
	update+=1;
	if (update > 1) update = 0;

	TIM_SR(TIM2) &= ~TIM_SR_UIF; /* Clear interrrupt flag. */
}

void tim3_isr(void)
{
	/* Update Time and Sync Calculations Here */
	updateTime(&timer, 1);

	TIM_SR(TIM3) &= ~TIM_SR_UIF; /* Clear interrrupt flag. */
}

int main(void)
{
	clock_setup();
	gpio_setup();
	i2c_setup();
	nvic_setup();
	timer_setup();

	MPU_Init mpu;
	
	mpuSetup(I2C1, &mpu);

	/* Set sender parameter as 1 for master and 0 for slave */
	synchronizeControllers(I2C1, &timer, 0);

	while (1) {

		/* Update Rate For Sensors Set To 2 Hz */
		if (update) {
			readAccelerometer(I2C1, mpu.acc);
			readGyroscope(I2C1, mpu.gyro);
			readMagnetometer(I2C1, mpu.mag, mpu.magCalibration);
			madgwickQuaternionRefresh(mpu.q, &mpu, mpu.acc, mpu.gyro, mpu.mag);
			quarternionToEulerAngle(mpu.q, &mpu.pitch, &mpu.yaw, &mpu.roll);

			/* Synchronize with other microcontrollers as master */

		}
	}

	return 0;
}