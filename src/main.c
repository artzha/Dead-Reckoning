#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>


static void clock_setup(void)
{
	rcc_clock_setup_in_hse_16mhz_out_72mhz();
}

static void gpio_setup(void)
{
	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Set GPIO6/7 (in GPIO port B) to 'output push-pull' for the LEDs. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, GPIO7);
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

	/* If everything is configured -> enable the peripheral. */
	i2c_peripheral_enable(I2C1);
}

void mpu_setup() {
	uint8_t data[2] = { MPU_ADDR_PW_MGT_CFG, 0 };
	i2c_transfer7(I2C1, MPU_ADDR, data, 2, data, 0);

	data[0] = ACC_ADDR_CFG;
	data[1] = 0b00011000;
	i2c_transfer7(I2C1, MPU_ADDR, data, 2, data, 0);

	data[0] = GYRO_ADDR_CFG;
	data[1] = 0b00011000;
	i2c_transfer7(I2C1, MPU_ADDR, data, 2, data, 0);
}
	
void read_accelerometer (double* acc) {
	uint8_t raw[6];
	uint8_t addr[6] = { 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40 };
	int i = 0;
	while(i < 6) {
		i2c_transfer7(I2C1, MPU_ADDR, addr+i, 1, raw+i, 1);
		i++;
	}

	acc[0] = (int16_t)((int16_t)raw[0]<<8|raw[1]);
	acc[1] = (int16_t)((int16_t)raw[2]<<8|raw[3]);
	acc[2] = (int16_t)((int16_t)raw[4]<<8|raw[5]);

	acc[0] = acc[0]/2048.0;
	acc[1] = acc[1]/2048.0;
	acc[2] = acc[2]/2048.0;
}

int main(void)
{
	clock_setup();
	gpio_setup();
	i2c_setup();
	mpu_setup();

	double acc[3];

	while (1) {
		read_accelerometer(acc);
		int i;
		for (i = 0; i < 800000; i++)    /* Wait a bit. */
		 __asm__("nop");
	}

	return 0;
}