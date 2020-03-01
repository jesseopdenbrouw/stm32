/*
 * Test routines for using the VMA412 touchscreen Graphic LCD
 *
 * Note: pin assignment for STM32F446 Nucleo Board
 *
 * Note: the 8-bit 8080 CPU interface is used.

Software License Agreement (BSD License)

Copyright (c) 2020 Jesse op den Brouw. All rights reserved.

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

#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "glcd_ili9341_stm32.h"


/* The Nucleo Boards have an 8 MHz external clock source */
#if defined(HSE_VALUE)
#warning Make sure that HSE_VALUE is set to 8000000 (8 MHz)
#undef HSE_VALUE
#endif /* HSE_VALUE */
  /*!< Value of the External oscillator in Hz */
#define HSE_VALUE    ((uint32_t)8000000U)


#define USEMAXMHZ

/* Test for boards */
#if defined(STM32F446xx)
#define GLCD_RCC_M (420)
#elif defined(STM32F411xx)
#define GLCD_RCC_M (250)
#else
#undef USEMAXMHZ
#define GLCD_RCC_M (50)
#endif

int main(void) {
	 uint16_t i, j;
	 uint32_t ranx, rany, ranc;

#ifdef USEMAXMHZ

	/* 5 wait states */
	FLASH->ACR = (FLASH_ACR_LATENCY_5WS << FLASH_ACR_LATENCY_Pos);

	/* Caches enable, prefetch enable */
	FLASH->ACR |= (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN);

	RCC->CR |= RCC_CR_HSEBYP;

	/* Start the Main PLL to MAX MHz
	 * FREQ = ((HSE/M)*N)/P
	 * 1 MHz <= HSE/M <= 2 MHz
	 * 50 <= N <= 432      2 <= M <= 63    P = 2(0),4(1),6(2),8(3)
	 */
	RCC->PLLCFGR = (420<<RCC_PLLCFGR_PLLN_Pos) | (5<<RCC_PLLCFGR_PLLM_Pos) |
				   (1<<RCC_PLLCFGR_PLLP_Pos) | (1<<RCC_PLLCFGR_PLLSRC_Pos);

	/* Enable HSE, PLL */
	RCC->CR |= RCC_CR_HSEON | RCC_CR_PLLON;
	//RCC->CR |= RCC_CR_HSEON;

	/* Wait for HSE, PLL to become ready */
	while ((RCC->CR & RCC_CR_HSERDY) == 0);
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	/* APB2 clk div'd by 2, APB1 clk div'd by 4 */
	RCC->CFGR = (4<<RCC_CFGR_PPRE2_Pos) | (5<<RCC_CFGR_PPRE1_Pos);

	/* Select PLL clock */
	RCC->CFGR |= (2<<RCC_CFGR_SW_Pos);

#endif

	/* Must be called first to read the system clock frequency */
	SystemCoreClockUpdate();

	/* Initialize GLCD */
	/* !! MUST BE CALLED *AFTER* CLOCK SOURCE INITIALIZATION !! */
	glcd_init();

	/* At this point, the GLCD is ready */

	/* Plot some rather nice colors */
	glcd_plotrectfill(0, 26*0, 320, 26, 0x0000ff);
	glcd_plotrectfill(0, 26*1, 320, 26, 0x00007f);
	glcd_plotrectfill(0, 26*2, 320, 26, 0x00003f);

	glcd_plotrectfill(0, 26*3, 320, 26, 0xff0000);
	glcd_plotrectfill(0, 26*4, 320, 26, 0x7f0000);
	glcd_plotrectfill(0, 26*5, 320, 26, 0x3f0000);

	glcd_plotrectfill(0, 26*6, 320, 26, 0x00ff00);
	glcd_plotrectfill(0, 26*7, 320, 26, 0x007f00);
	glcd_plotrectfill(0, 26*8, 320, 26, 0x003f00);

	glcd_plotrectfill(0, 26*9, 320, 6, 0x000000);

	/* Plot random pixels */
	srand(0);
	ranx = rand();
	ranx = rand();

	for (i=0; i<65535; i++) {
	  ranx = rand()%320;
	  rany = rand()%240;
	  ranc = rand();

	  glcd_plotpixel(ranx, rany, ranc);
	}

	/* Clear the screen with some colors */
	glcd_cls(0x000000);
	glcd_cls(0x0000ff);
	glcd_cls(0x00ff00);
	glcd_cls(0x00ffff);
	glcd_cls(0xff0000);
	glcd_cls(0xff00ff);
	glcd_cls(0xffff00);
	glcd_cls(0xffffff);
	glcd_cls(0x000000);

	/* Plot the sine and cosine */
	const double angle = 2*M_PI/320;

	for (i=0; i<320; i++) {
		j = 120.0-(sin(i*angle)*119.0+0.5);
		glcd_plotpixel(i, j, 0xffffff);
		j = 120.0-(cos(i*angle)*119.0+0.5);
		glcd_plotpixel(i, j, 0xff0000);
	}

	/* Plot some strings */
	glcd_plotstring(0,32, "Hallo Kirsten, hoe is het met jou?\x01\x02\x03\x04", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_CONDENSED);
	glcd_plotstring(0,40, "Hallo Kirsten, hoe is het met jou?\x01\x02\x03\x04", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
	glcd_plotstring(0,48, "Hallo Kirsten, hoe is het met jou?\x01\x02\x03\x04", GLCD_COLOR_YELLOW, GLCD_COLOR_YELLOW, GLCD_STRING_WIDE);

	/* Plot rectangles */
	glcd_plotrectfill(221, 129, 58, 28, GLCD_COLOR_MAGENTA);
	glcd_plotrect(220, 128, 60, 30, GLCD_COLOR_YELLOW);

	/* Plot circle */
	glcd_plotcircle(100, 100, 30, GLCD_COLOR_WHITE);

	/* Plot the complete character table */
	for (i=0; i<8; i++) {
		for (j=0; j<32; j++) {
			glcd_plotchar( 10+j*6, 120+i*9, i*32+j, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK);
		}
	}

	glcd_delay_ms(2000);

	glcd_cls(GLCD_COLOR_THUASGREEN);

	glcd_cls(GLCD_COLOR_WHITE);
	glcd_plotbitmap(0, 72, GLCD_THUAS_DEFAULT_BITMAP, 320, 96, GLCD_COLOR_THUASGREEN, GLCD_COLOR_WHITE);
	//glcd_plotbitmap(0, 0, GLCD_THUAS_DEFAULT_BITMAP_SMALL, 160, 48, GLCD_COLOR_THUASGREEN, GLCD_COLOR_WHITE);
	glcd_plotstring(100, 200, "Department of Electrical Engineering", GLCD_COLOR_THUASGREEN, GLCD_COLOR_WHITE, GLCD_STRING_NORMAL);

	/* Invert the display */
	for (i=1; i<=10; i++) {
		glcd_inversion(GLCD_DISPLAY_INVERSION_ON);
		glcd_delay_ms(50);
		glcd_inversion(GLCD_DISPLAY_INVERSION_OFF);
		glcd_delay_ms(50);
	}

	/* Set the display to idle */
	glcd_idle(GLCD_DISPLAY_IDLE_ON);
	glcd_delay_ms(2000);
	glcd_idle(GLCD_DISPLAY_IDLE_OFF);

	/* Display off and on */
	glcd_display(GLCD_DISPLAY_OFF);
	glcd_delay_ms(2000);
	glcd_display(GLCD_DISPLAY_ON);

	while (1)
	{
	  __NOP();
	}
}
