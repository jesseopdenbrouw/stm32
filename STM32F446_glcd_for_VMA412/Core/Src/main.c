/*
 * Test routines for using the Velleman VMA412 touchscreen Graphic LCD
 *
 * Note: pin assignment for STM32F446 Nucleo Board
 *
 * Note: the 8-bit 8080 CPU interface is used.

Software License Agreement (BSD License)

Version: 0.2
Date: 2020/06/29

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

/* Note: set compiler optimization to -O0 for debug */
/* Note: set compiler optimization to -Ofast for fastest result */

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "main.h"
#include "glcd_ili9341_stm32.h"


/* The Nucleo Boards have an 8 MHz external clock source */
#if defined(HSE_VALUE)
#warning Make sure that HSE_VALUE is set to 8000000 (8 MHz)
#endif /* HSE_VALUE */

#define USEMAXMHZ /* Use maximum frequency */
#define MEASUREMENT /* Do time measurements of the GLCD functions */


#ifdef MEASUREMENT
/* For measurement system */
#define START_CLOCK() \
	SysTick->LOAD  = (uint32_t)(0xffffffL);       /* set reload register */ \
	SysTick->VAL   = 0UL;                         /* Load the SysTick Counter Value */ \
	SysTick->CTRL  = SysTick_CTRL_ENABLE_Msk;

#define GET_CLOCK(A) \
		SysTick->CTRL = 0; \
		clockval[A] = ((0xffffff - SysTick->VAL)/prescaler)

#define SETUP_CLOCK() \
	volatile uint32_t clockval[20] = {0}; \
	volatile uint32_t prescaler = SystemCoreClock/1000000UL/8UL; \
	const char *clocknames[20] ={ "Display initialize:     ", \
			                      "Vertical scroll:        ", \
								  "Clear screen:           ", \
								  "Circle Flood Fill:      ", \
								  "Triangle Flood Fill:    ", \
								  "Arc Flood Fill:         ", \
								  "Vertical scroll:        ", \
								  "Plot bitmap THUAS:      ", \
								  "Plot circle (100x):     ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Console 53 chars (10x): ", \
								  "50 ms delay:            " \
								  };

#define PRINT_CLOCK() {\
		char string[20]; \
		glcd_printconsole("\fTiming of the GLCD functions:\n\n"); \
		for (int i=0; i<20; i++) { \
			glcd_printconsole(clocknames[i]); \
			sprintf(string, "%6lu.%03lu ms\n", clockval[i]/1000UL, clockval[i]%1000UL); \
			glcd_printconsole(string); \
		} \
		}
#else
#define START_CLOCK()
#define GET_CLOCK(A)
#define SETUP_CLOCK()
#define PRINT_CLOCK()
#endif

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
    char sprintfbuf[80];
	/* To monitor speeds */

#ifdef USEMAXMHZ

	/* 5 wait states */
 	FLASH->ACR = (FLASH_ACR_LATENCY_5WS << FLASH_ACR_LATENCY_Pos);

	/* Caches enable, prefetch enable */
	FLASH->ACR |= (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN);

	/* Select HSE bypass to use clock oscillator */
	RCC->CR |= RCC_CR_HSEBYP;

	/* Start the Main PLL to MAX MHz
	 * FREQ = ((HSE/M)*N)/P
	 * 1 MHz <= HSE/M <= 2 MHz
	 * 50 <= N <= 432      2 <= M <= 63    P = 2(0),4(1),6(2),8(3)
	 */
	RCC->PLLCFGR = (GLCD_RCC_M<<RCC_PLLCFGR_PLLN_Pos) | (5<<RCC_PLLCFGR_PLLM_Pos) |
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

	/* Update the system clock frequency variable
	 * Can be omitted since glcd_init() also performs
	 * the update */
	SystemCoreClockUpdate();

	/* For speed measurements */
	SETUP_CLOCK();

	/* Initialize GLCD */
	/* !! MUST BE CALLED *AFTER* CLOCK SOURCE INITIALIZATION !! */

	START_CLOCK();
	glcd_init();
	GET_CLOCK(0);

	/* Tweak the length of the read/write pulse, USE WITH CARE */
	/* Argument must be > 0. Smaller values means faster reads/writes */
	//glcd_set_write_pulse_delay(3);

	/* At this point, the GLCD is ready */

/* Set to 0 to skip the demo and start working on your own project */
#if 1

	do {
	/* Set the rotation
	 * Note: printing currently works correct for 0 and 90 degrees
	 * Note: not all demos work correctly on rotation
	 */
	glcd_setrotation(GLCD_SCREEN_ROT0);

	/* Print version and date */
	glcd_cls(GLCD_COLOR_BLACK);
	glcd_plotstring(10, 48, GLCD_VERSION, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
	glcd_plotstring(10, 58, "Driver for VELLEMAN VMA412", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
	glcd_plotstring(10, 68, "Using 8-bit 8080 interface", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	/* Using good old sprintf to print the clock speed
	 * Really it should be snprintf to be on the save side */
	sprintf(sprintfbuf, "Clock speed: %lu", SystemCoreClock);
	glcd_plotstring(10, 100, sprintfbuf, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	printf("Hallo");

	glcd_delay_ms(3000);

	/* Console based printing */
	glcd_printconsole("\f");
    for (i=0; i<10; i++) {
    	glcd_printconsole("Hello\n");
    }
	START_CLOCK();
	for (i=0; i<10; i++) {
		glcd_printconsole("12345678901234567890123456789012345678901234567890ABC\r");
	}
    GET_CLOCK(18);
    for (int i=0; i<10; i++) {
    	glcd_printconsole("Hello\n");
    }
	glcd_printconsole("123456789012345678901234567890123456789012345678901234567890\n");

	START_CLOCK();
    glcd_printconsole("\nConsole based print routines\n\nAnd another line\nAnd another one\b\b\bline\n\b\b\bhello\n");
    GET_CLOCK(1);

	glcd_delay_ms(3000);

	/* Plot some rather nice colors */
	glcd_plotrectfill(0, 26*0, 320, 26, 0xff0000);
	glcd_plotrectfill(0, 26*1, 320, 26, 0x7f0000);
	glcd_plotrectfill(0, 26*2, 320, 26, 0x3f0000);

	glcd_plotrectfill(0, 26*3, 320, 26, 0x00ff00);
	glcd_plotrectfill(0, 26*4, 320, 26, 0x007f00);
	glcd_plotrectfill(0, 26*5, 320, 26, 0x003f00);

	glcd_plotrectfill(0, 26*6, 320, 26, 0x0000ff);
	glcd_plotrectfill(0, 26*7, 320, 26, 0x00007f);
	glcd_plotrectfill(0, 26*8, 320, 26, 0x00003f);

	glcd_plotrectfill(0, 26*9, 320, 6, GLCD_COLOR_BLACK);

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
	START_CLOCK();
	glcd_cls(GLCD_COLOR_BLUE);
	GET_CLOCK(2);
	glcd_cls(GLCD_COLOR_GREEN);
	glcd_cls(GLCD_COLOR_CYAN);
	glcd_cls(GLCD_COLOR_RED);
	glcd_cls(GLCD_COLOR_MAGENTA);
	glcd_cls(GLCD_COLOR_YELLOW);
	glcd_cls(GLCD_COLOR_WHITE);
	glcd_cls(GLCD_COLOR_BLACK);

	/* Plot all the grey scales */
	for (i=0; i<256; i++) {
		glcd_plotverticalline(i, 0, 240, (i<<16)+(i<<8)+i);
	}
	glcd_plotstring(10, 32, "Plotting grey scales", GLCD_COLOR_YELLOW, GLCD_COLOR_YELLOW, GLCD_STRING_NORMAL);

	glcd_delay_ms(2000);

	glcd_cls(GLCD_COLOR_BLACK);

	/* Plot the sine and cosine */
	const float angle = 2*M_PI/320.0f;

	/* We use sinf and cosf because of the hardware FPU */
	for (i=0; i<320; i++) {
		j = (uint16_t) 120.0f-(sinf((float)i*angle)*119.0f+0.5f);
		glcd_plotpixel(i, j, GLCD_COLOR_WHITE);
		j = (uint16_t) 120.0f-(cosf((float)i*angle)*119.0f+0.5f);
		glcd_plotpixel(i, j, GLCD_COLOR_RED);
	}

	/* Plot some strings */
	glcd_plotstring(0,32, "Hallo Kirsten, hoe is het met jou?\x01\x02\x03\x04", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_CONDENSED);
	glcd_plotstring(0,40, "Hallo Kirsten, hoe is het met jou?\x01\x02\x03\x04", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
	glcd_plotstring(0,48, "Hallo Kirsten, hoe is het met jou?\x01\x02\x03\x04", GLCD_COLOR_YELLOW, GLCD_COLOR_YELLOW, GLCD_STRING_WIDE);

	/* Plot rectangles */
	glcd_plotrectfill(221, 129, 58, 28, GLCD_COLOR_MAGENTA);
	glcd_plotrect(220, 128, 60, 30, GLCD_COLOR_YELLOW);

	/* Plot circle */
	START_CLOCK();
	for (i=0; i<99; i++) {
		glcd_plotcircle(100, 100, 100, GLCD_COLOR_WHITE);
	}
	GET_CLOCK(8);

	/* Plot the complete character table */
	for (i=0; i<8; i++) {
		for (j=0; j<32; j++) {
			glcd_plotchar( 10+j*6, 120+i*9, i*32+j, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK);
		}
	}

	glcd_delay_ms(2000);

	/* Plot a circle and fill it */
	glcd_cls(GLCD_COLOR_BLACK);
	glcd_plotcircle(50, 100, 30, GLCD_COLOR_WHITE);
#ifdef GLCD_USE_FLOOD_FILL
	START_CLOCK();
	glcd_floodfill(22, 100, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK);
	GET_CLOCK(3);
#endif

	/* Plot a triangle and fill it */
	glcd_plotline(200, 200, 150, 150, GLCD_COLOR_WHITE);
	glcd_plotline(200, 200, 240, 130, GLCD_COLOR_WHITE);
	glcd_plotline(240, 130, 150, 150, GLCD_COLOR_WHITE);
#ifdef GLCD_USE_FLOOD_FILL
	START_CLOCK();
	glcd_floodfill(210, 180, GLCD_COLOR_RED, GLCD_COLOR_BLACK);
	GET_CLOCK(4);
	__NOP();
#endif

	/* Plot an arc */
#ifdef GLCD_USE_ARC
	glcd_plotarc(100, 100, 100, 0.0, 60.0, GLCD_COLOR_MAGENTA);
	glcd_plotline(100, 100, 200, 100, GLCD_COLOR_MAGENTA);
	glcd_plotline(100, 100, 150, 187, GLCD_COLOR_MAGENTA);
#ifdef GLCD_USE_FLOOD_FILL
	START_CLOCK();
	glcd_floodfill(120, 105, GLCD_COLOR_GREEN, GLCD_COLOR_BLACK);
	GET_CLOCK(5);
#endif
#endif

	glcd_delay_ms(3000);

	glcd_plotstring(10, 220, "Performing vertical shift in software", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
	for (int nr=0; nr<240/8; nr++) {
		START_CLOCK();
		glcd_scrollvertical(8);
		GET_CLOCK(6);
	}

	glcd_cls(GLCD_COLOR_THUASGREEN);


	/* Plot the THUAS bitmap */
	glcd_cls(GLCD_COLOR_WHITE);
	START_CLOCK();
	glcd_plotbitmap(0, 72, GLCD_THUAS_DEFAULT_BITMAP, 320, 96, GLCD_COLOR_THUASGREEN, GLCD_COLOR_WHITE);
	GET_CLOCK(7);
	//glcd_plotbitmap(0, 0, GLCD_THUAS_DEFAULT_BITMAP_SMALL, 160, 48, GLCD_COLOR_THUASGREEN, GLCD_COLOR_WHITE);
	glcd_plotstring(100, 200, "Department of Electrical Engineering", GLCD_COLOR_THUASGREEN, GLCD_COLOR_WHITE, GLCD_STRING_NORMAL);

	/* Invert the display */
	for (i=1; i<=10; i++) {
		glcd_inversion(GLCD_DISPLAY_INVERSION_ON);
		glcd_delay_ms(100);
		glcd_inversion(GLCD_DISPLAY_INVERSION_OFF);
		glcd_delay_ms(100);
	}

	/* Set the display to idle */
	glcd_idle(GLCD_DISPLAY_IDLE_ON);
	glcd_delay_ms(2000);
	glcd_idle(GLCD_DISPLAY_IDLE_OFF);

	/* Display off and on */
	glcd_display(GLCD_DISPLAY_OFF);
	glcd_delay_ms(2000);
	glcd_display(GLCD_DISPLAY_ON);
    glcd_delay_ms(1000);

    glcd_plotstring(10, 20, "Brought to you by:", GLCD_COLOR_THUASGREEN, GLCD_COLOR_WHITE, GLCD_STRING_NORMAL);
	glcd_delay_ms(1000);

    /* Measurement of 50 ms */
    START_CLOCK();
    glcd_delay_ms(50);
    GET_CLOCK(19);

    PRINT_CLOCK();
    __NOP();

#ifdef MEASUREMENT
	glcd_putchar('\n');
    for (i=9; i>0; i--) {
    	glcd_putchar(i+0x30);
    	glcd_printconsole(" seconds left\r");
    	glcd_delay_ms(1000);
    }
#else
	glcd_delay_ms(10000);
#endif
	} while (1);

#endif

	while (1)
	{
	  __NOP();
	}
}
