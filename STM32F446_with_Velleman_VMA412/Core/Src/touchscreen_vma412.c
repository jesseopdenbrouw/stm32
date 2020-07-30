/*
 * Routines for using the Velleman VMA412 touchscreen Graphic LCD
 *
 * Note: pin assignment for STM32F446 Nucleo Board
 *
 * Note: the 8-bit 8080 CPU interface is used.

Software License Agreement (BSD License)

Version: 0.1rc4
Date: 2020/07/30

Copyright (c) 2020 Jesse op den Brouw.  All rights reserved.

Portions based on :
 Copyright (c) 2012 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

 */


#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdint.h>

#include "touchscreen_vma412.h"

/* GPIO A(1), B(2), C(4) USED */
/* Really should be done by a function that reads the used ports */
static const uint32_t TOUCH_GPIO_USED = 7;

/* The Analog Channels using Arduino notation
 * Only works for ADC1 and ADC2, because ADC3 doesn't share
 * PA4 and PB0 with ADC1/ADC2
 */
typedef struct {
	GPIO_TypeDef *pGPIO; // One of GPIOA, GPIOB or GPIOC
	uint32_t channel; // One of 1 .. 18
	uint32_t portbit;   // One of 0 .. 15
} analogchannel_t;

static const analogchannel_t analogchannels[] =
{
	{GPIOA,  0, 0},    // Arduino ANA channel 0 == PA0
	{GPIOA,  1, 1},    // Arduino ANA channel 1 == PA1
	{GPIOA,  4, 4},    // Arduino ANA channel 2 == PA4
	{GPIOB,  8, 0},    // Arduino ANA channel 3 == PB0
	{GPIOC, 11, 1},    // Arduino ANA channel 4 == PC1
	{GPIOC, 10, 0},    // Arduino ANA channel 5 == PC0
    {NULL,   0, 0}
};

/* The digital channels using Arduino notation */
typedef struct {
	GPIO_TypeDef *pGPIO; // One of GPIOA, GPIOB or GPIOC
	uint32_t channel;    // Not used
	uint32_t portbit;    // One of 0 .. 15
} digitalchannel_t;

static const digitalchannel_t digitalchannels[] =
{
	{GPIOA,  0,  3},    // Arduino DIG channel 0 == PA3
	{GPIOA,  0,  2},    // Arduino DIG channel 1 == PA2
	{GPIOA,  0, 10},    // Arduino DIG channel 2 == PA10
	{GPIOB,  0,  3},    // Arduino DIG channel 3 == PB3
	{GPIOB,  0,  5},    // Arduino DIG channel 4 == PB5
	{GPIOB,  0,  4},    // Arduino DIG channel 5 == PB4
	{GPIOB,  0, 10},    // Arduino DIG channel 6 == PB10
	{GPIOA,  0,  8},    // Arduino DIG channel 7 == PA8
	{GPIOA,  0,  9},    // Arduino DIG channel 8 == PA9 --> GLCD_D0
	{GPIOC,  0,  7},    // Arduino DIG channel 9 == PC7 --> GLCD_D1
	{GPIOB,  0,  6},    // Arduino DIG channel 10 == PB6
	{GPIOA,  0,  7},    // Arduino DIG channel 11 == PA7
	{GPIOA,  0,  6},    // Arduino DIG channel 12 == PA6
	{GPIOA,  0,  5},    // Arduino DIG channel 13 == PA5
	// Needs more digital channels
    {NULL,   0, 0}
};

/* Measured with VMA412 */
#define XP analogchannels[3]   /* LCD_CS PB0 */
#define XM digitalchannels[9]  /* LCD_D1 PC7 */
#define YP digitalchannels[8]  /* LCD_D0 PA9 */
#define YM analogchannels[2]   /* LCD_RS PA4 */

//#define SD_CLK digitalchannel[13]

/* The ADC to use */
static ADC_TypeDef *pADC;
/* Buffer for the samples */
static uint32_t samples[TOUCH_SAMPLES];

/* Delay before analog inputs have settled */

static uint32_t readdelay;

/* Function touchscreen_delay_init
 * Initializes the delay routine
 * @private
 * @in: void
 * @out: void
 */
static void touchscreen_delay_init(void) {
	SystemCoreClockUpdate();
	readdelay = 400UL*(SystemCoreClock/1000UL)/3000000UL+1UL;
}

/* Function touchscreen_delay_read
 * Creates a delay for the GLCD read functions
 * @private
 * @in: none
 * @out: void
 */
static void touchscreen_delay(void) {

		uint32_t ns = readdelay;

	    asm volatile ("mov r3, %[ns] \n\t"
				  ".Lglcdhu%=: \n\t"
				  "subs r3, #1 \n\t"
				  "bne .Lglcdhu%= \n\t"
                  :
                  : [ns] "r" (ns)
                  : "r3", "cc"
				 );
}

/* Taken from the Adafruit library
 * Insertion sort is slow, but it doesn't use
 * recursive calls. And the number of points
 * are low.
 */
/* Function touchscreen_insert_sort
 * Sorts a number of samples ascending
 * @private
 * @in: array -- the array
 * @in: size  -- size of the array
 * @out: void
 */
#ifdef __GNUC__
/* Keep the compiler from complaining if not used */
__attribute__((used))
#endif
static void touchscreen_insert_sort(uint32_t array[], uint16_t size) {
  uint8_t j;
  int save;

  for (int i = 1; i < size; i++) {
    save = array[i];
    for (j = i; j >= 1 && save < array[j - 1]; j--)
      array[j] = array[j - 1];
    array[j] = save;
  }
}

/* Initialize touchscreen with dedicated ADC */
/* Function touchscreen_init
 * Initializes the touchscreen
 * @public
 * @in: used_ADC --> ADC handle
 * @out: 1 == success, 0 == failure
 */
uint32_t touchscreen_init(ADC_TypeDef *used_ADC) {

	touchscreen_delay_init();

	pADC = used_ADC;

	/* Enable IO port A, B, C Clocks */
	/* Should be done by a routine thats reads the GPIOs
	 * from all the PortBit variables and makes an OR mask*/
	RCC->AHB1ENR &= ~TOUCH_GPIO_USED;
	RCC->AHB1ENR |= TOUCH_GPIO_USED;

	/* Enable ADCx clock */
#ifdef ADC1
	if (pADC == ADC1) {
		RCC->APB2ENR |= (1<<RCC_APB2ENR_ADC1EN_Pos);
#endif
#ifdef ADC2
	} else if (pADC == ADC2) {
		RCC->APB2ENR |= (1<<RCC_APB2ENR_ADC2EN_Pos);
#endif
/* ADC can't be used because two of the pins are not available on ADC3 */
//#ifdef ADC3
//	} else if (pADC == ADC3) {
//		RCC->APB2ENR |= (1<<RCC_APB2ENR_ADC3EN_Pos);
//	} else {
//#endif
		/* Unknown ADC */
		pADC = NULL;
		return 0;
#ifdef ADC
	}
#endif

	/* ADC Clock, prescaler = 8 (slowest speed), for all ADCs */
	/* The ADC doesn't have to be that fast for the touchscreen functions, but maybe for other functions */
	ADC->CCR = (3 << ADC_CCR_ADCPRE_Pos);

	/* ADCx active */
	pADC->CR2 = (1<<ADC_CR2_ADON_Pos);

	/* Wait for the ADCx to start up (3 us) */
	/* Really, this should be an assembler subroutine */
    uint32_t counter = (3 * (SystemCoreClock / 1000000U));
    while(counter != 0U)
    {
      counter--;
    }

	/* n-bit samples, 0 = 12, 1 = 10, 2 = 8, 3 = 6 */
    /* 10 bit samples used, as in an Arduino */
	pADC->CR1 &= (3 << ADC_CR1_RES_Pos);
	pADC->CR1 = (1 << ADC_CR1_RES_Pos);

	/* Sample time = 3 cycles, for channel 0 */
    /* Mazidi page 206, Fig. 7-9B */
	pADC->SMPR2 = (0<<ADC_SMPR2_SMP0_Pos);

	/* One conversion in sequence */
	/* These are the L bits that select the total sequence */
	/* 0 = one in sequence, 1 = two seq etc */
	pADC->SQR1 = (0<<ADC_SQR1_L_Pos);

	return 1;
}

/* Set the ADC speed after it is initialized */
/* Function touchscreen_setadcspeed
 * Sets the ADC speed for all ADCs
 * @public
 * @in: speed -- the speed
 * @out: void
 */
void touchscreen_setadcspeed(uint32_t speed) {

	/* No ADC configured */
	if (pADC == NULL) {
		return;
	}

	speed &= 0x3;

	ADC->CCR &= ~(3 << ADC_CCR_ADCPRE_Pos);
	ADC->CCR |= (speed << ADC_CCR_ADCPRE_Pos);
}

/* Function touchscreen_readrawx
 * Reads X coordinate of touchscreen (RAW)
 * @public
 * @in: void
 * @out: raw X-value
 */
uint32_t touchscreen_readrawx(void) {

	uint32_t MODERA, MODERB, MODERC;

	/* No ADC configured */
	if (pADC == NULL) {
		return 0;
	}

	/* n-bit samples, 0 = 12, 1 = 10, 2 = 8, 3 = 6 */
    /* 10 bit samples used, as in an Arduino */
	pADC->CR1 &= (3 << ADC_CR1_RES_Pos);
	pADC->CR1 = (1 << ADC_CR1_RES_Pos);

	/* Save directions */
	MODERA = GPIOA->MODER;
	MODERB = GPIOB->MODER;
	MODERC = GPIOC->MODER;

	/* Select Arduino compatible ADC Channel 0/1/2/... for ADCx */
	pADC->SQR3 = (YM.channel << ADC_SQR3_SQ1_Pos);

	/* YP is not used, set to input PA9 */
	(YP.pGPIO)->MODER &= ~(3<<(YP.portbit<<1));
	//(YP.pGPIO)->MODER |= (3<<(YP.portbit<<1));

	/* Set the GPIO port/pin to Analog Function */
	//(YM.pGPIO)->MODER &= ~(3<<(YM.portbit<<1));
	(YM.pGPIO)->MODER |= (3<<(YM.portbit<<1));


	// XP is output, out is high LCD_CS PB0 */
	(XP.pGPIO)->MODER &= ~(3<<(XP.portbit<<1));
	(XP.pGPIO)->MODER |= (1<<(XP.portbit<<1));
	(XP.pGPIO)->BSRR   = (1<<(XP.portbit));

	/* XM is output, out is low  LCD_D1 PC7 */
	(XM.pGPIO)->MODER &= ~(3<<(XM.portbit<<1));
	(XM.pGPIO)->MODER |= (1<<(XM.portbit<<1));
	(XM.pGPIO)->BSRR   = (1<<(XM.portbit+16));

	/* Wait a bit for analog inputs have stabilized */
	touchscreen_delay();

	for (int i=0; i <TOUCH_SAMPLES; i++) {
		/* Start conversion */
		pADC->CR2 |= (1<<ADC_CR2_SWSTART_Pos);

		/* Wait for conversion complete */
		while ((pADC->SR & (1<<ADC_SR_EOC_Pos)) == 0);

		samples[i] = pADC->DR;
	}

	/* Restore directions */
	GPIOA->MODER = MODERA;
	GPIOB->MODER = MODERB;
	GPIOC->MODER = MODERC;

	touchscreen_insert_sort(samples, TOUCH_SAMPLES);

	return samples[TOUCH_SAMPLES/2];
}

/* Function touchscreen_readrawy
 * Reads Y coordinate of touchscreen (RAW)
 * @public
 * @in: void
 * @out: raw Y-value
 */
uint32_t touchscreen_readrawy(void) {

	uint32_t MODERA, MODERB, MODERC;

	/* No ADC configured */
	if (pADC == NULL) {
		return 0;
	}

	/* n-bit samples, 0 = 12, 1 = 10, 2 = 8, 3 = 6 */
    /* 10 bit samples used, as in an Arduino */
	pADC->CR1 &= (3 << ADC_CR1_RES_Pos);
	pADC->CR1 = (1 << ADC_CR1_RES_Pos);

	/* Save directions */
	MODERA = GPIOA->MODER;
	MODERB = GPIOB->MODER;
	MODERC = GPIOC->MODER;

	/* Select Arduino compatible ADC Channel 0/1/2/... for ADCx */
	pADC->SQR3 = (XP.channel << ADC_SQR3_SQ1_Pos);

	/* XM is not used input PC7 */
	(XM.pGPIO)->MODER &= ~(3<<(XM.portbit<<1));
	//(XM.pGPIO)->MODER |= (0<<(XM.portbit<<1));

	/* Set the GPIO port/pin to Analog Function */
	/* Use an bitwise OR PB0  */
	//(XP.pGPIO)->MODER &= ~(3<<(XP.portbit<<1));
	(XP.pGPIO)->MODER |= (3<<(XP.portbit<<1));

	// YP is output, out is high PA9
	//(YP.pGPIO)->OSPEEDR |= (3<<(YP.portbit<<1));
	(YP.pGPIO)->MODER &= ~(3<<(YP.portbit<<1));
	(YP.pGPIO)->MODER |= (1<<(YP.portbit<<1));
	(YP.pGPIO)->BSRR   = (1<<(YP.portbit));

	/* YM is output, out is low PA4 */
	//(YM.pGPIO)->OSPEEDR |= (3<<(YM.portbit<<1));
	(YM.pGPIO)->MODER &= ~(3<<(YM.portbit<<1));
	(YM.pGPIO)->MODER |= (1<<(YM.portbit<<1));
	(YM.pGPIO)->BSRR   = (1<<(YM.portbit+16));

	/* Wait a bit for analog inputs have stabilized */
	touchscreen_delay();

	for (int i=0; i <TOUCH_SAMPLES; i++) {
		/* Start conversion */
		pADC->CR2 |= (1<<ADC_CR2_SWSTART_Pos);

		/* Wait for conversion complete */
		while ((pADC->SR & (1<<ADC_SR_EOC_Pos)) == 0);

		samples[i] = pADC->DR;
	}

	/* Restore directions */
	GPIOA->MODER = MODERA;
	GPIOB->MODER = MODERB;
	GPIOC->MODER = MODERC;

	/* Find the mean of the array */
	touchscreen_insert_sort(samples, TOUCH_SAMPLES);

	return samples[TOUCH_SAMPLES/2];
}

/* Function touchscreen_pressure
 * Reads pressure (RAW)
 * @public
 * @in: void
 * @out: raw pressure value
 */
uint32_t touchscreen_pressure(void) {

	uint32_t p1, p2;
	uint32_t MODERA, MODERB, MODERC;

	/* No ADC configured */
	if (pADC == NULL) {
		return 0;
	}

	/* n-bit samples, 0 = 12, 1 = 10, 2 = 8, 3 = 6 */
    /* 10 bit samples used, as in an Arduino */
	pADC->CR1 &= (3 << ADC_CR1_RES_Pos);
	pADC->CR1 = (1 << ADC_CR1_RES_Pos);

	/* Save directions */
	MODERA = GPIOA->MODER;
	MODERB = GPIOB->MODER;
	MODERC = GPIOC->MODER;

	/* XP analog is input  */
//	(XP.pGPIO)->MODER &= ~(3<<(XP.portbit<<1));
	(XP.pGPIO)->MODER |= (3<<(XP.portbit<<1));

	/* YM analog is input */
//	(YM.pGPIO)->MODER &= ~(3<<(YM.portbit<<1));
	(YM.pGPIO)->MODER |= (3<<(YM.portbit<<1));

	// XM is output, out is high
	(YP.pGPIO)->MODER &= ~(3<<(YP.portbit<<1));
	(YP.pGPIO)->MODER |= (1<<(YP.portbit<<1));
	(YP.pGPIO)->BSRR   = (1<<(YP.portbit));

	/* YP is output, out is low*/
	(XM.pGPIO)->MODER &= ~(3<<(XM.portbit<<1));
	(XM.pGPIO)->MODER |= (1<<(XM.portbit<<1));
	(XM.pGPIO)->BSRR   = (1<<(XM.portbit+16));

	/* Wait a bit for analog inputs have stabilized */
	touchscreen_delay();

	/* Read XP first */
	pADC->SQR3 = (XP.channel << ADC_SQR3_SQ1_Pos);

	/* Start conversion */
	for (int i=0; i <TOUCH_SAMPLES; i++) {
		/* Start conversion */
		pADC->CR2 |= (1<<ADC_CR2_SWSTART_Pos);

		/* Wait for conversion complete */
		while ((pADC->SR & (1<<ADC_SR_EOC_Pos)) == 0);

		samples[i] = pADC->DR;
	}

	/* Sort the readings */
	touchscreen_insert_sort(samples, TOUCH_SAMPLES);

	/* Take the mean */
	p1 = samples[TOUCH_SAMPLES/2];


	/* Now read YM */
	pADC->SQR3 = (YM.channel << ADC_SQR3_SQ1_Pos);

	/* Start conversion */
	for (int i=0; i <TOUCH_SAMPLES; i++) {
		/* Start conversion */
		pADC->CR2 |= (1<<ADC_CR2_SWSTART_Pos);

		/* Wait for conversion complete */
		while ((pADC->SR & (1<<ADC_SR_EOC_Pos)) == 0);

		samples[i] = pADC->DR;
	}

	/* Sort the readings */
	touchscreen_insert_sort(samples, TOUCH_SAMPLES);

	/* Take the mean */
	p2 = samples[TOUCH_SAMPLES/2];

	/* Restore directions */
	GPIOA->MODER = MODERA;
	GPIOB->MODER = MODERB;
	GPIOC->MODER = MODERC;

//	if (p2<p1) {
//		return 1023-(p1-p2);
//	}
	return 1023 - (p2 - p1);
}

/* Taken from to Arduino library, uses floats */
/* Maps raw touchscreen value to a screen value
 * @public
 * @in: value --> the raw value
 * @in: tlow  --> touchscreen raw lowest value
 * @in: thigh --> touchscreen raw higest value
 * @in: slow  --> screen raw lowest value
 * @in: shigh --> screen raw higest value
 * @out: remapped value (note: signed integer, so may be negative)
 */
int32_t touchscreen_map(uint32_t value, uint32_t tlow, uint32_t thigh, uint32_t slow, uint32_t shigh) {

	/* We use floats here because the STM32F has hardware support for floats */
//	float slope, start;

//	/* Make sure that thigh != low */
//	if (thigh!=tlow) {
//		slope  = ((float)shigh-(float)slow)/((float)thigh-(float)tlow);
//		start = - slope*(float)tlow;
//		return slope*value + start;
//	}

	if (thigh != tlow) {
		return ((float)value - (float)(tlow))*((float)shigh - (float)slow) / ((float)thigh - (float)tlow) + (float)slow + (float)0.5f;
	}

	/* If thigh == tlow, we divide by 0 and that is not possible */
	return INT_MIN;
}

/* Function touchscreen_pressed
 * Is touchscreen pressed or not?
 * @public
 * @in: p --> the raw pressure value
 * @out: 0 is not pressed, !0 if pressed
 */
uint32_t touchscreen_ispressed(uint32_t p) {
	return (p>=TOUCH_PRESSURE_LOW && p<=TOUCH_PRESSURE_HIGH);
}
