/*
 *
 *  DAC output example
 *
 *  This example writes a sine wave to the DAC1 output.
 *
 *  12 bits are used.
 *
 *  Because of the internal op amp buffer, the maximum and
 *  minimum values of the DAC are not written correctly.
 *
 *  As by datasheet, the minimum value is 0.2 V above 0 V
 *  and the maximum value is 0.2 V below VCC (positive power).
 *  This means that an amplification of the sine wave is
 *  about 1820. This calculates according to the following
 *
 *  Min = 0x0e0
 *  Max = 0xf1c
 *  Offset = 0x800
 *  The offset is 2048.
 *
 *  So the minimum amplification is 0x800 - 0x0e0 = 0x720
 *  And the maximum amplification is 0xf1c - 0x800 = 0x71c
 *
 *  This means that a sine can reproduced at best with
 *  amplification 0x71c and that is 1820 decimal.
 *
 *  Note: the DAC output settling time is 6 us max.
 *
 * Setting the M of the PLL to 336, gives a sine wave
 * of about 109 kHz.
 * If you take less samples, the waveform with degrade
 * to a triangle because of the DAC speed.
 *
 */

#include "main.h"
#include <math.h>

#define USEMAXMHZ

/* Test for boards */
#if defined(STM32F446xx)
#define GLCD_RCC_M (360)
#else
/* No DAC available */
#undef USEMAXMHZ
#endif

#define LENGTH (256)
#define BASEANGLE (2*M_PI/LENGTH)

int main(void) {

	uint32_t i;
	uint32_t sintab[LENGTH];

	/* Calculate the sine table */
	for (i=0; i<LENGTH; i++) {
		// No output buffer
		//sintab[i] = 2047.5*sin(BASEANGLE*i)+2048.0;
		sintab[i] = 1820.0*sin(BASEANGLE*i)+2048.0;
	}

#ifdef USEMAXMHZ

	/* For the correct procedure to enable max core frequency,
	 * see RM0390, page 97 (Entering Over-drive Mode)
	 */

	/* 5 wait states */
 	FLASH->ACR = (FLASH_ACR_LATENCY_5WS << FLASH_ACR_LATENCY_Pos);

	/* Caches enable, prefetch enable */
	FLASH->ACR |= (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN);

	/* Enable the power system */
	do {
		__IO uint32_t tmpreg = 0x00U;
		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
		/* Delay after an RCC peripheral clock enabling */
		tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);
		UNUSED(tmpreg);
	} while(0U);

	/* Select Scale 1 voltage regulator (max clock frequency) */
	do {
		__IO uint32_t tmpreg = 0x00U;
		MODIFY_REG(PWR->CR, PWR_CR_VOS, PWR_REGULATOR_VOLTAGE_SCALE1);
		/* Delay after an RCC peripheral clock enabling */
		tmpreg = READ_BIT(PWR->CR, PWR_CR_VOS);
		UNUSED(tmpreg);
	} while(0U);

	/* Select HSE bypass to use clock oscillator */
	RCC->CR |= RCC_CR_HSEBYP;

	/* Start the Main PLL to MAX MHz
	 * FREQ = ((HSE/M)*N)/P
	 * 1 MHz <= HSE/M <= 2 MHz
	 * 50 <= N <= 432      2 <= M <= 63    P = 2(0),4(1),6(2),8(3)
	 */
	RCC->PLLCFGR = (GLCD_RCC_M<<RCC_PLLCFGR_PLLN_Pos) | (4<<RCC_PLLCFGR_PLLM_Pos) |
				   (1<<RCC_PLLCFGR_PLLP_Pos) | (1<<RCC_PLLCFGR_PLLSRC_Pos);

	/* Enable HSE, PLL */
	RCC->CR |= RCC_CR_HSEON | RCC_CR_PLLON;
	//RCC->CR |= RCC_CR_HSEON;

	/* Wait for HSE, PLL to become ready */
	while ((RCC->CR & RCC_CR_HSERDY) == 0);
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	/* Enable the Over-drive to extend the clock frequency to 180 MHz */
	//PWR->CR |= PWR_CR_ODEN;
	__HAL_PWR_OVERDRIVE_ENABLE();

	/* Wait for Over-drive enable */
	// while (!(PWR->CSR & (PWR_CSR_ODRDY)) == (PWR_CSR_ODRDY));
	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_ODRDY)) {}

	/* Enable the Over-drive switch */
	//(*(__IO uint32_t *) CR_ODSWEN_BB = ENABLE); // bit banding
	//PWR->CR |= PWR_CR_ODSWEN;                   // normal way
	__HAL_PWR_OVERDRIVESWITCHING_ENABLE();

	/* Wait for Over-drive ready */
	// while (!(PWR->CSR & (PWR_CSR_ODSWRDY)) == (PWR_CSR_ODSWRDY));
	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_ODSWRDY)) {}


	/* APB2 clk div'd by 2, APB1 clk div'd by 4, AHB div'd by 1, use PLL */
	RCC->CFGR = (4<<RCC_CFGR_PPRE2_Pos) | (5<<RCC_CFGR_PPRE1_Pos) | (0<<RCC_CFGR_HPRE_Pos)| (2<<RCC_CFGR_SW_Pos);

	/* Wait for PLL to lock on */
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

#endif


	/* Update the SystemCoreClock variable */
	SystemCoreClockUpdate();

	volatile uint32_t dummy = SystemCoreClock;

	/* Enable the GPIOA clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	/* Select pin PA4 analog mode (mode 3, output of DAC1) */
	GPIOA->MODER |= GPIO_MODER_MODER4;

	/* Enable DAC clock */
	RCC->APB1ENR |= RCC_APB1LPENR_DACLPEN;
	/* Enable DAC1 */
	DAC->CR |= DAC_CR_EN1;

	/* Generate a sine wave of 100 kHz on pin PA4 */
	while (1) {
		for (i=0; i<LENGTH; i++) {
			/* Write sine table sample to the Data Holding Register */
			/* for 12 bits, right aligned */
			DAC->DHR12R1 = sintab[i];
		}
	}

	return 0;
}
