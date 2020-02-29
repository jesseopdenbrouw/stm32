

#include "main.h"
#include <math.h>

#define LENGTH (256)
#define BASEANGLE (2*M_PI/LENGTH)

int main(void) {

	uint32_t i;
	uint32_t sintab[LENGTH];

	/* Calculate the sine table */
	for (i=0; i<LENGTH; i++) {
		sintab[i] = 2047.0*sin(BASEANGLE*i)+2048.0;
	}

	/* Select the HSE clock (8 MHz) */
	RCC->CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
	RCC->CFGR |= RCC_CFGR_SW_0;

	/* Update the SystemCoreClock variable */
	SystemCoreClockUpdate();

	/* Enable the GPIOA clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	/* Select pin PA4 analog mode (mode 3, output of DAC1) */
	GPIOA->MODER |= GPIO_MODER_MODER4;

	/* Enable DAC clock */
	RCC->APB1ENR |= RCC_APB1LPENR_DACLPEN;
	/* Enable DAC1 */
	DAC->CR |= DAC_CR_EN1;

	/* Generate a sine wave of 1 kHz on pin PA4 */
	while (1) {
		for (i=0; i<LENGTH; i++) {
			/* Write sine table sample to the Data Holding Register */
			/* for 12 bits, right aligned */
			DAC->DHR12R1 = sintab[i];
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
		}
	}

	return 0;
}
