/*
 * Demo for using the Velleman VMA412 touchscreen Graphic LCD
 *
 * Note: pin assignment for STM32F446 Nucleo Board
 *
 * Note: the 8-bit 8080 CPU interface is used.

Software License Agreement (BSD License)

Version: 0.1
Date: 2020/07/15

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

#include <stdio.h>
#include <math.h>

#include <glcd_ili9341_vma412.h>
#include <touchscreen_vma412.h>

#include "main.h"


/* Test for boards */
#if defined(STM32F446xx)
#define GLCD_RCC_M (420)
#elif defined(STM32F411xx)
#define GLCD_RCC_M (250)
#else
#undef USEMAXMHZ
#define GLCD_RCC_M (50)
#endif

/* Use maximum frequency */
#define USEMAXMHZ

 /* Do time measurements of the GLCD functions */
#define MEASUREMENT



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
	const char *clocknames[20] ={ "GLCD initialize:        ", \
			                      "Clear screen (8x):      ", \
								  "Plot pixel (100000x):   ", \
								  "Plot circle (100x):     ", \
								  "Plot rectangle (1000x): ", \
								  "Plot filled rect (10x): ", \
								  "Flood fill circle:      ", \
								  "Flood fill triangle:    ", \
								  "Flood fill arc:         ", \
								  "Vertical scroll:        ", \
								  "Plot bitmap:            ", \
								  "Printing 53 chars (10x):", \
								  "Print + vertical shift: ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Not used                ", \
								  "Not used                ", \
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

void demo_touchscreen(void);

void demo_glcd(void);

int main(void) {

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
	//RCC->CFGR = (4<<RCC_CFGR_PPRE2_Pos) | (5<<RCC_CFGR_PPRE1_Pos);
	/* APB2 clk div'd by 2, APB1 clk div'd by 1, USE PLL */
	RCC->CFGR = (4<<RCC_CFGR_PPRE2_Pos) | (0<<RCC_CFGR_PPRE1_Pos) | (2<<RCC_CFGR_SW_Pos);

#endif

    /* For printing clock speed */
	char buffer[40];

	uint16_t x, y;

	/* Update the system clock frequency variable
	 * Can be omitted since glcd_init() also performs
	 * the update */
	SystemCoreClockUpdate();


	/* Initialize GLCD */
	/* !! MUST BE CALLED *AFTER* CLOCK SOURCE INITIALIZATION !! */
	glcd_init();

	/* Tweak the length of the read/write pulse, USE WITH CARE */
	/* Argument must be > 0. Smaller values means faster reads/writes */
	//glcd_set_write_pulse_delay(3);

	/* At this point, the GLCD is ready */

	/* Initialize the touchscreen system */
	/* Select one of ADC1 and ADC2 */
	touchscreen_init(ADC1);

	/* At this point, the touchscreen is ready */

	while (1) {

		/* Clear the screen */
		glcd_cls(GLCD_COLOR_BLACK);

		/* Print driver info */
		glcd_plotstring(10, 48, GLCD_VERSION, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
		glcd_plotstring(10, 58, TOUCH_VERSION, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

		snprintf(buffer, sizeof buffer, "Clock speed: %lu", SystemCoreClock);
		glcd_plotstring(10, 80, buffer, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

		glcd_plotrectfill(20, 120, 20, 20, GLCD_COLOR_YELLOW);
		glcd_plotstring(50, 126, "Touch rectangle to start GLCD demo", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

		glcd_plotrectfill(20, 150, 20, 20, GLCD_COLOR_YELLOW);
		glcd_plotstring(50, 156, "Touch rectangle to start touchscreen demo", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

		/* Get a coordinate from the touchscreen */
		while (1) {

			/* Wait for the screen to be touched */
			while (!touchscreen_pressed(touchscreen_pressure())) {}

			x = touchscreen_readx();
			x = touchscreen_map(x, TOUCH_LEFT, TOUCH_RIGHT, 0, glcd_getwidth());

			y = touchscreen_ready();
			y = touchscreen_map(y, TOUCH_BOTTOM, TOUCH_TOP, 0, glcd_getheight());

			if (x>=20 && x<=20+20 & y>=120 && y<=120+20) {
				demo_glcd();
				break;
			}
			if (x>=20 && x<=20+20 & y>=150 && y<=150+20) {
				demo_touchscreen();
				break;
			}
		}
	}

	return 0;
}

void demo_touchscreen(void) {

	glcd_color_t color = 0;

	uint32_t xraw, yraw, p;
	int32_t x, y;

	glcd_printconsole("\fTouch rectangle on the right to exit");
	glcd_plotrectfill(glcd_getwidth()-20, 0, 20, 20, GLCD_COLOR_WHITE);

	while (1) {
		/* Read raw X coordinate and map it to screen coordinate */
		xraw = touchscreen_readx();
		x = touchscreen_map(xraw, TOUCH_LEFT, TOUCH_RIGHT, 0, glcd_getwidth());

		/* Read raw Y coordinate and map it to screen coordinate */
		yraw = touchscreen_ready();
		y = touchscreen_map(yraw, TOUCH_BOTTOM, TOUCH_TOP, 0, glcd_getheight());

		/* Read raw pressure */
		p = touchscreen_pressure();

		color++;

		if (touchscreen_pressed(p)) {
			/* Touched in the upper right corner? Then clear screen */
			if (x>glcd_getwidth()-20 && y<20) {
				break;
			} else {
				glcd_plotcircle(x, y, 3, color);
			}
		}
	}

}

void demo_glcd(void) {

	uint32_t i, j;
	/* For speed measurements */
	SETUP_CLOCK();

	/* Initialize GLCD */
	/* !! MUST BE CALLED *AFTER* CLOCK SOURCE INITIALIZATION !! */

	START_CLOCK();
	glcd_init();
	GET_CLOCK(0);

	/* Clear the screen with some colors */
	START_CLOCK();
	glcd_cls(GLCD_COLOR_BLUE);
	glcd_cls(GLCD_COLOR_GREEN);
	glcd_cls(GLCD_COLOR_CYAN);
	glcd_cls(GLCD_COLOR_RED);
	glcd_cls(GLCD_COLOR_MAGENTA);
	glcd_cls(GLCD_COLOR_YELLOW);
	glcd_cls(GLCD_COLOR_WHITE);
	glcd_cls(GLCD_COLOR_BLACK);
	GET_CLOCK(1);

	glcd_printconsole("\fPlotting 100000 pixels...");

	START_CLOCK();
	for (i = 0; i < 100000; i++) {
		glcd_plotpixel(100,100, GLCD_COLOR_WHITE);
	}
	GET_CLOCK(2);

	glcd_printconsole("\fPlotting 100 circles...\n");

	START_CLOCK();
	for (i = 0; i < 100; i++) {
		glcd_plotcircle(100,100, 80, GLCD_COLOR_WHITE);
	}
	GET_CLOCK(3);

	glcd_printconsole("Plotting 1000 rectangles...\n");

	START_CLOCK();
	for (i = 0; i < 1000; i++) {
		glcd_plotrect(20,100, 200, 100, GLCD_COLOR_MAGENTA);
	}
	GET_CLOCK(4);

	glcd_printconsole("Plotting 10 filled rectangles...\n");

	START_CLOCK();
	for (i = 0; i < 10; i++) {
		glcd_plotrectfill(20,100, 200, 100, GLCD_COLOR_RED);
	}
	GET_CLOCK(5);

	glcd_printconsole("\fFilling objects...\n");

    START_CLOCK();
	glcd_plotcircle(100, 100, 30, GLCD_COLOR_WHITE);
#ifdef GLCD_USE_FLOOD_FILL
	glcd_floodfill(100, 100, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK);
    GET_CLOCK(6);
#endif

	/* Plot a triangle and fill it */
	glcd_plotline(200, 200, 150, 150, GLCD_COLOR_WHITE);
	glcd_plotline(200, 200, 240, 130, GLCD_COLOR_WHITE);
	glcd_plotline(240, 130, 150, 150, GLCD_COLOR_WHITE);
#ifdef GLCD_USE_FLOOD_FILL
	START_CLOCK();
	glcd_floodfill(210, 180, GLCD_COLOR_RED, GLCD_COLOR_BLACK);
	GET_CLOCK(7);
#endif

	/* Plot an arc */
#ifdef GLCD_USE_ARC
	glcd_plotarc(100, 100, 100, 0.0, 60.0, GLCD_COLOR_MAGENTA);
	glcd_plotline(100, 100, 200, 100, GLCD_COLOR_MAGENTA);
	glcd_plotline(100, 100, 150, 187, GLCD_COLOR_MAGENTA);
#ifdef GLCD_USE_FLOOD_FILL
	START_CLOCK();
	glcd_floodfill(130, 110, GLCD_COLOR_GREEN, GLCD_COLOR_BLACK);
	GET_CLOCK(8);
#endif
#endif
	glcd_delay_ms(2000);

	glcd_plotstring(10, 220, "Performing vertical scroll in software", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
	for (int nr=0; nr<240/8; nr++) {
		START_CLOCK();
		glcd_scrollvertical(8);
		GET_CLOCK(9);
	}

	glcd_printconsole("\fMiscellaneous");

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

	/* Plot the complete character table */
	for (i=0; i<8; i++) {
		for (j=0; j<32; j++) {
			glcd_plotchar( 10+j*6, 120+i*9, i*32+j, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK);
		}
	}

	glcd_delay_ms(5000);

	/* Plot the THUAS bitmap */
	glcd_cls(GLCD_COLOR_WHITE);
	START_CLOCK();
	glcd_plotbitmap(0, 72, GLCD_THUAS_DEFAULT_BITMAP, 320, 96, GLCD_COLOR_THUASGREEN, GLCD_COLOR_WHITE);
	GET_CLOCK(10);
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

	/* Console based printing */
	glcd_printconsole("\fConsole based printing");
    for (i=0; i<10; i++) {
    	glcd_printconsole("Hello\n");
    }
	START_CLOCK();
	for (i=0; i<10; i++) {
		glcd_printconsole("12345678901234567890123456789012345678901234567890ABC\r");
	}
    GET_CLOCK(11);
    for (int i=0; i<10; i++) {
    	glcd_printconsole("Hello\n");
    }
	glcd_printconsole("123456789012345678901234567890123456789012345678901234567890\n");

	START_CLOCK();
    glcd_printconsole("\nConsole based print routines\n\nAnd another line\nAnd another one\b\b\bline\n\b\b\bhello\n");
    GET_CLOCK(12);

	glcd_delay_ms(3000);

	/* Plot all the grey scales */
	for (i=0; i<256; i++) {
		glcd_plotverticalline(i, 0, 240, (i<<16)+(i<<8)+i);
	}
	glcd_printconsole("Plotting grey scales");

	glcd_delay_ms(2000);


	/* Measurement of 50 ms */
    START_CLOCK();
    glcd_delay_ms(50);
    GET_CLOCK(19);

    PRINT_CLOCK();

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
}
