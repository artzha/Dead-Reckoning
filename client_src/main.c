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
volatile uint8_t reading = 0;
volatile uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
volatile uint8_t orientation[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t orienKey = 0;
volatile uint8_t bufKey = 0;
volatile uint8_t num_cycles = 0;
volatile uint8_t mode = 0;
Time timer = {0, 0, {0}, 0, 0}; // initialize timer values

/* Declare mpu vars for interrupt usage */
volatile float intPitch, intRoll, intYaw = 0;

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
	nvic_set_priority(NVIC_I2C1_EV_IRQ, 1);
}

static void i2c_setup(void)
{
	/* Enable clocks for I2C1 and I2C2 and AFIO. */
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_I2C2);
	rcc_periph_clock_enable(RCC_AFIO);

	/* Set alternate functions for the SCL and SDA pins of I2C2. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		      GPIO_I2C1_SCL | GPIO_I2C1_SDA | GPIO_I2C2_SCL | GPIO_I2C2_SDA);

	/* Disable the I2C before changing any configuration. */
	i2c_peripheral_disable(I2C1);
	i2c_peripheral_disable(I2C2);

	/* APB1 is running at 36MHz. */
	i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);
	i2c_set_clock_frequency(I2C2, I2C_CR2_FREQ_36MHZ);

	/* 400KHz - I2C Fast Mode */
	i2c_set_fast_mode(I2C1);
	i2c_set_fast_mode(I2C2);

	/*
	 * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
	 * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
	 * Datasheet suggests 0x1e.
	 */
	i2c_set_ccr(I2C1, 0x1e);
	i2c_set_ccr(I2C2, 0x1e);

	/*
	 * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
	 * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
	 * Incremented by 1 -> 11.
	 */
	i2c_set_trise(I2C1, 0x0b);
	i2c_set_trise(I2C2, 0x0b);

	/*
	 * This is our slave address - needed only if we want to receive from
	 * other masters.
	 */
	i2c_set_own_7bit_slave_address(I2C1, 0x32);
	i2c_set_own_7bit_slave_address(I2C2, 0x32);

	/* 
	 * This allows master to interrupt slave at any time to compare 
	 * calculation and synchronize time data
	 */
	i2c_enable_interrupt(I2C1, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
	i2c_enable_interrupt(I2C2, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(I2C1);
	i2c_peripheral_enable(I2C2);

	// slave needs to acknowledge on receiving bytes
	// set it after enabling Peripheral i.e. PE = 1
	i2c_enable_ack(I2C1);
	i2c_enable_ack(I2C2);
}

void i2c1_ev_isr(void)
{
   	uint32_t sr1, sr2;

   	sr1 = I2C_SR1(I2C1);

	// Address matched (Slave)
	if (sr1 & I2C_SR1_ADDR)
	{
		if(timer.mode == 0 || timer.mode == 1) {
			/* Update time store millis|seconds MSB then LSB*/
			buf[0] = (timer.millis>>8) & 0xFF;
			buf[1] = timer.millis & 0xFF;
			buf[2] = (timer.seconds>>24) & 0xFF;
			buf[3] = (timer.seconds>>16) & 0xFF;
			buf[4] = (timer.seconds>>8) & 0xFF;
			buf[5] = timer.seconds & 0xFF;
		} else {

			/* Reset orientation index if reached end of array */
			uint8_t i = 0;
			while(i < 3) {
				/* Convert and store current orientation measurements */
				uint32_t measurement;
				switch(i) {
					case 0:
						if (intPitch < 0) {
							measurement = (uint32_t)(-intPitch * 1000);
						} else {
							measurement = (uint32_t)(intPitch * 1000);
						}
						orientation[5*i] = intPitch < 0 ? 0 : 1;
						break;
					case 1:
						if (intRoll < 0) {
							measurement = (uint32_t)(-intRoll * 1000);
						} else {
							measurement = (uint32_t)(intRoll * 1000);
						}
						orientation[5*i] = intRoll < 0 ? 0 : 1;
						break;
					case 2:
						if (intYaw < 0) {
							measurement = (uint32_t)(-intYaw * 1000);
						} else {
							measurement = (uint32_t)(intYaw * 1000);
						}
						orientation[5*i] = intYaw < 0 ? 0 : 1;
						break;
					default:
						break;
				}
				/* Encode 0 for negative and 1 for positive measurement */

				orientation[(5*i)+1] = (measurement>>24) & 0xFF;
				orientation[(5*i)+2] = (measurement>>16) & 0xFF;
				orientation[(5*i)+3] = (measurement>>8) & 0xFF;
				orientation[(5*i)+4] = measurement & 0xFF;
				i++;
			}
		}

		//Clear the ADDR sequence by reading SR2.
		sr2 = I2C_SR2(I2C1);
		(void) sr2;
	}
	// Receive buffer not empty
	else if (sr1 & I2C_SR1_RxNE)
	{
		if (timer.mode == 0 || timer.mode == 1) {
			/* reset reading index if at end of offset array */
			if (reading == 5) {
				reading = 0;
			}
			/* Receive offset amount from master and calibrate */
			timer.offset[reading] = i2c_get_data(I2C1);
			reading++;
		} else {
			// TODO Load master data as needed 
		}
	}
	// Transmit buffer empty & Data byte transfer not finished
	else if ((sr1 & I2C_SR1_TxE) && !(sr1 & I2C_SR1_BTF))
	{
		if (timer.mode == 0 || timer.mode == 1) {
			/* Reset transmit buffer index if at end of buffer array */
			if (bufKey == 6) {
				bufKey = 0;
			}
			i2c_send_data(I2C1, *(buf + bufKey));
			bufKey++;
		} else {
			if (orienKey == 15) {
				orienKey = 0;
			}
			i2c_send_data(I2C1, *(orientation + orienKey));
			orienKey++;
		}
	}
	// done by master by sending STOP
	//this event happens when slave is in Recv mode at the end of communication
	else if (sr1 & I2C_SR1_STOPF)
	{
		i2c_peripheral_enable(I2C1);

		if (timer.mode == 0 || timer.mode == 1) {
			/* Subroutine on end slave reception */
			int32_t amount = bytesToTime(timer.offset+1);
			/* update own time once last byte in offset buffer has been received */
			if (reading == 5) {
				amount = (timer.offset[0] == 0) ? amount*(-1) : amount;
				/* Update time on slave with offset */
				timer.delay = -amount;
				updateTime(&timer, timer.delay);

				/* reset offset buffer after saving to delay */
				uint8_t i = 0;
				while(i < 5) {
					timer.offset[i] = 0;
					i++;
				}
				/* Increment mode until in orientation mode */
				timer.mode++;
			}
		} else {
			// TODO Handle master commands if needed
		}
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
	
	mpuSetup(I2C2, &mpu);

	while (1) {

		/* Update Rate For Sensors Set To 2 Hz */
		if (update) {
			readAccelerometer(I2C2, mpu.acc);
			readGyroscope(I2C2, mpu.gyro);
			readMagnetometer(I2C2, mpu.mag, mpu.magCalibration);
			madgwickQuaternionRefresh(mpu.q, &mpu, mpu.acc, mpu.gyro, mpu.mag);
			quarternionToEulerAngle(mpu.q, &mpu.pitch, &mpu.yaw, &mpu.roll);

			/* Synchronize with other microcontrollers as master */
			intPitch = mpu.pitch;
			intRoll = mpu.roll;
			intYaw = mpu.yaw;
		}
	}

	return 0;
}