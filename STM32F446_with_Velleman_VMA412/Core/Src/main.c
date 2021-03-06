/*
 * Demo for using the Velleman VMA412 touchscreen Graphic LCD
 *
 * Note: pin assignment for STM32F446 Nucleo Board
 *
 * Note: the 8-bit 8080 CPU interface is used.

Software License Agreement (BSD License)

Version: 0.1rc6
Date: 2020/08/09

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

#if !defined(STM32F446xx) && !defined(STM32F411xE)
#warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#warning ! Only tested with STM32F446 Nucleo Board !
#warning ! Only tested with STM32F411 Nucleo Board !
#warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#endif

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "glcd_vma412.h"
#include "touchscreen_vma412.h"

#include "main.h"


/* Test for boards */
#if defined(STM32F446xx)
#define GLCD_RCC_M (360)
#elif defined(STM32F411xE)
#define GLCD_RCC_M (250)
#else
#undef USEMAXMHZ
#define GLCD_RCC_M (50)
#endif

/* Use maximum frequency */
#define USEMAXMHZ

 /* Do time measurements of the GLCD functions */
#define MEASUREMENT

/* Let's see if we can plot an 256-color indexed picture */
#define HAVE_DOG

/* For testing purposes, if you know what your're doing */
//#define USE_TEST


#ifdef MEASUREMENT
/* For measurement system */
#define START_CLOCK() \
	SysTick->LOAD  = (uint32_t)(0xffffffL);       /* set reload register */ \
	SysTick->VAL   = 0UL;                         /* Load the SysTick Counter Value */ \
	SysTick->CTRL  = SysTick_CTRL_ENABLE_Msk;

#define GET_CLOCK(A) \
		SysTick->CTRL = 0; \
		clockval[A] = ((0xffffff - SysTick->VAL)/prescaler);

#define SETUP_CLOCK() \
	volatile uint32_t clockval[20] = {0}; \
	volatile uint32_t prescaler = SystemCoreClock/1000000UL/8UL; \
	char *clocknames[20] ={ "GLCD initialize:        ", /*  0 */ \
			                      "Clear screen (8x):      ", /*  1 */ \
								  "Plot pixel (100000x):   ", /*  2 */ \
								  "Plot circle (100x):     ", /*  3 */ \
								  "Plot rectangle (1000x): ", /*  4 */ \
								  "Plot filled rect (50x): ", /*  5 */ \
								  "Plot filled circ (100x):", /*  6 */ \
								  "Plot filled tria (100x):", /*  7 */ \
								  "Flood fill circle:      ", /*  8 */ \
								  "Flood fill triangle:    ", /*  9 */ \
								  "Flood fill circ sector: ", /* 10 */ \
								  "Vertical scroll:        ", /* 11 */ \
								  "Printing 53 chars (10x):", /* 12 */ \
								  "Print + vertical shift: ", /* 13 */ \
								  "Plot 256-color picture: ", /* 14 */ \
								  "Plot THUAS bitmap:      ", /* 15 */ \
								  "Not used                ", /* 16 */ \
								  "Not used                ", /* 17 */ \
								  "Not used                ", /* 18 */ \
								  "100 ms delay:           "  /* 19 */ \
								  };

#define PRINT_CLOCK() {\
		char string[20]; \
		glcd_puts("\fTiming of the GLCD functions:\n\n"); \
		for (int i=0; i<20; i++) { \
			glcd_puts(clocknames[i]); \
			sprintf(string, "%6lu.%03lu ms\n", clockval[i]/1000UL, clockval[i]%1000UL); \
			glcd_puts(string); \
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
void demo_rotation(void);
void demo_characterset(void);
#ifdef USE_TEST
void test(void);
#endif

#ifdef HAVE_DOG
extern const uint8_t dog_map[];
#endif

#ifdef GLCD_HAVE_THUAS_BITMAPS
/* Use the THUAS bitmaps */
extern const uint8_t glcd_thuas_map[];
extern const uint8_t glcd_thuas_small_map[];
#endif

/* Use alternate fonts */
extern const GFXfont FreeSerif12pt7b;
extern const GFXfont FreeMono12pt7b;

int main(void) {

	SystemInit();

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

#if defined(STM32F446xx)

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

#endif

	/* APB2 clk div'd by 2, APB1 clk div'd by 4, AHB div'd by 1, use PLL */
	RCC->CFGR = (4<<RCC_CFGR_PPRE2_Pos) | (5<<RCC_CFGR_PPRE1_Pos) | (0<<RCC_CFGR_HPRE_Pos)| (2<<RCC_CFGR_SW_Pos);

	/* Wait for PLL to lock on */
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

#endif

    /* For printing clock speed */
	char buffer[40];

	uint16_t x, y, raw;

	/* Update the system clock frequency variable
	 * Can be omitted since glcd_init() also performs
	 * the update */
	SystemCoreClockUpdate();


	/* Initialize GLCD */
	/* !! MUST BE CALLED *AFTER* CLOCK SOURCE INITIALIZATION !! */
	glcd_init();

	/* Tweak the length of the read/write pulse, USE WITH CARE */
	/* Argument must be > 0. Smaller values means faster reads/writes */
//	glcd_set_write_pulse_delay(3);
//	glcd_set_read_pulse_delay(10);

	/* At this point, the GLCD is ready */

	/* Initialize the touchscreen system */
	/* Select one of ADC1 and ADC2 */
	touchscreen_init(ADC1);

	/* At this point, the touchscreen is ready */

	while (1) {

		/* Clear the screen */
		glcd_cls(GLCD_COLOR_BLACK);

		/* Print driver info */
		glcd_plotstring(10, 28, GLCD_VERSION, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
		glcd_plotstring(10, 38, TOUCH_VERSION, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

		snprintf(buffer, sizeof buffer, "Clock speed: %lu", SystemCoreClock);
		glcd_plotstring(10, 60, buffer, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

		glcd_plotrectfill(20, 100, 20, 20, GLCD_COLOR_YELLOW);
		glcd_plotstring(50, 106, "Touch rectangle to start GLCD demo", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

		glcd_plotrectfill(20, 130, 20, 20, GLCD_COLOR_YELLOW);
		glcd_plotstring(50, 136, "Touch rectangle to start touchscreen demo", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

		glcd_plotrectfill(20, 160, 20, 20, GLCD_COLOR_YELLOW);
		glcd_plotstring(50, 166, "Touch rectangle to start rotation demo", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

		glcd_plotrectfill(20, 190, 20, 20, GLCD_COLOR_YELLOW);
		glcd_plotstring(50, 196, "Touch rectangle to show character set", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

#ifdef USE_TEST
		glcd_plotrectfill(20, 220, 20, 20, GLCD_COLOR_YELLOW);
		glcd_plotstring(50, 226, "Test", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
#endif

		/* Get a coordinate from the touchscreen */
		while (1) {

			/* Wait for the screen to be touched */
			while (!touchscreen_ispressed(touchscreen_pressure())) {}

			raw = touchscreen_readrawx();
			x = touchscreen_map(raw, TOUCH_LEFT, TOUCH_RIGHT, 0, glcd_getwidth());

			raw = touchscreen_readrawy();
			y = touchscreen_map(raw, TOUCH_BOTTOM, TOUCH_TOP, 0, glcd_getheight());

			if (x>=20 && x<=20+20 && y>=100 && y<=100+20) {
				demo_glcd();
				break;
			}
			if (x>=20 && x<=20+20 && y>=130 && y<=130+20) {
				demo_touchscreen();
				break;
			}
			if (x>=20 && x<=20+20 && y>=160 && y<=160+20) {
				demo_rotation();
				break;
			}
			if (x>=20 && x<=20+20 && y>=190 && y<=190+20) {
				demo_characterset();
				break;
			}
#ifdef USE_TEST
			if (x>=20 && x<=20+20 && y>=220 && y<=220+20) {
				test();
				break;
			}
#endif
		}
	}

	return 0;
}

/* Touch screen drawing demo */
void demo_touchscreen(void) {

	glcd_color_t color = 0;

	uint32_t xraw, yraw, p;
	int32_t x, y;

	uint32_t touched = 0;

	glcd_puts("\fTouch rectangle on the right to exit");
	glcd_plotrectfill(glcd_getwidth()-20, 0, 20, 20, GLCD_COLOR_WHITE);

	/* Wait until the touchscreen is untouched */
	while (!touchscreen_ispressed(touchscreen_pressure())) {}
	while (touchscreen_ispressed(touchscreen_pressure())) {}

	/* Remove text */
	glcd_plotrectfill(0, 0, glcd_getwidth()-20, 20, GLCD_COLOR_BLACK);

	/* Now read in x and y and pressure */
	while (1) {

		if (touched == 0) {
			/* Read raw pressure, X and Y */
			p = touchscreen_pressure();
			xraw = touchscreen_readrawx();
			yraw = touchscreen_readrawy();

			/* Map to screen coordinates */
			x = touchscreen_map(xraw, TOUCH_LEFT, TOUCH_RIGHT, 0, glcd_getwidth());
			y = touchscreen_map(yraw, TOUCH_BOTTOM, TOUCH_TOP, 0, glcd_getheight());

			if (touchscreen_ispressed(p)) {
				/* Pressed before x and y are read, so valid (hopefully) */
				touched = 1;
			}
		} else { /* touched == 1 */

			/* Read raw X, Y and pressure */
			xraw = touchscreen_readrawx();
			yraw = touchscreen_readrawy();
			p = touchscreen_pressure();

			/* Map to screen coordinates */
			x = touchscreen_map(xraw, TOUCH_LEFT, TOUCH_RIGHT, 0, glcd_getwidth());
			y = touchscreen_map(yraw, TOUCH_BOTTOM, TOUCH_TOP, 0, glcd_getheight());

			if (!touchscreen_ispressed(p)) {
				/* Unpressed  after x and y are read, so invalid (hopefully) */
				touched = 0;
			}
		}

		color += 0x010104;

		if (touched) {
			/* Touched in the upper right corner? Then clear screen */
			if (x>glcd_getwidth()-20 && y<20) {
				break;
			} else {
				glcd_plotcircle(x, y, 3, color);
			}
		}
	}
}

/* GLCD demo, no touch needed */
void demo_glcd(void) {

	uint32_t i, j;
	glcd_color_t color = 0x0000e0;

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

	glcd_puts("\fPlotting 100000 pixels...\n");

	START_CLOCK();
	for (i = 0; i < 100000; i++) {
		glcd_plotpixel(100,100, GLCD_COLOR_WHITE);
	}
	GET_CLOCK(2);

	glcd_puts("Plotting 100 circles...\n");

	START_CLOCK();
	for (i = 0; i < 100; i++) {
		glcd_plotcircle(100,100, 80, GLCD_COLOR_WHITE);
	}
	GET_CLOCK(3);

	glcd_puts("Plotting 1000 rectangles...\n");

	START_CLOCK();
	for (i = 0; i < 1000; i++) {
		glcd_plotrect(20,100, 200, 100, GLCD_COLOR_MAGENTA);
	}
	GET_CLOCK(4);

	glcd_puts("Plotting 50 filled rectangles...\n");

	START_CLOCK();
	for (i = 0; i < 50; i++) {
		glcd_plotrectfill(20,100, 200, 100, color);
		color += 0x030104;
	}
	GET_CLOCK(5);

	glcd_puts("Plotting 100 filled circles...\n");
	START_CLOCK();
	for (i = 0; i < 100; i++) {
		glcd_plotcirclefill(60, 180, 50, color);
		color += 0x030104;
	}
	GET_CLOCK(6);

	glcd_puts("Plotting 100 filled triangles...\n");
	START_CLOCK();
	for (i = 0; i < 100; i++) {
		glcd_plottrianglefill(100, 100, 150, 150, 80, 180, color);
		color += 0x030104;
	}
	GET_CLOCK(7);

	glcd_puts("\fFilling objects...\n");

    START_CLOCK();
	glcd_plotcircle(100, 100, 30, GLCD_COLOR_WHITE);
#ifdef GLCD_USE_FLOOD_FILL
	glcd_floodfill(100, 100, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK);
    GET_CLOCK(8);
#endif

	/* Plot a triangle and fill it */
	glcd_plottriangle(200, 200, 150, 150, 240, 120, GLCD_COLOR_WHITE);
#ifdef GLCD_USE_FLOOD_FILL
	START_CLOCK();
	glcd_floodfill(200, 180, GLCD_COLOR_RED, GLCD_COLOR_BLACK);
	GET_CLOCK(9);
#endif

	/* Plot an arc */
#ifdef GLCD_USE_ARC
	glcd_plotarc(100, 100, 100, 0.0, 60.0, GLCD_COLOR_MAGENTA);
	glcd_plotline(100, 100, 200, 100, GLCD_COLOR_MAGENTA);
	glcd_plotline(100, 100, 150, 187, GLCD_COLOR_MAGENTA);
#ifdef GLCD_USE_FLOOD_FILL
	START_CLOCK();
	glcd_floodfill(130, 110, GLCD_COLOR_GREEN, GLCD_COLOR_BLACK);
	GET_CLOCK(10);
#endif
#endif
	glcd_delay_ms(2000);

	glcd_plotstring(10, 220, "Performing vertical scroll in software", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
	for (int nr=0; nr<240/8; nr++) {
		START_CLOCK();
		glcd_scrollvertical(8);
		GET_CLOCK(11);
	}

	glcd_puts("\fMiscellaneous printing and plotting");

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

	glcd_setcharsize(2, 2);
	glcd_plotstring(0,68, "Big", GLCD_COLOR_YELLOW, GLCD_COLOR_YELLOW, GLCD_STRING_WIDE);

	glcd_setcharsize(3, 3);
	glcd_plotstring(0,98, "Bigger", GLCD_COLOR_YELLOW, GLCD_COLOR_YELLOW, GLCD_STRING_WIDE);

	glcd_setcharsize(1, 1);

	/* Plot the complete character table */
	for (i=0; i<8; i++) {
		for (j=0; j<32; j++) {
			glcd_plotchar( 10+j*6, 150+i*9, i*32+j, GLCD_COLOR_YELLOW, GLCD_COLOR_YELLOW);
		}
	}

	glcd_delay_ms(5000);

#ifdef GLCD_USE_ALTERNATIVE_FONTS
	/* Now for some alternative fonts */
	glcd_cls(GLCD_COLOR_MAROON);

	glcd_plotstring(0, 0, "Now for some alternative fonts...", GLCD_COLOR_PINK, GLCD_COLOR_PINK, GLCD_STRING_NORMAL);

	/* Use address of font handle */
	glcd_setfont(&FreeMono12pt7b);

	glcd_plotstringwithfont(0, 100, "This is a monotype font", GLCD_COLOR_PINK, GLCD_STRING_CONDENSED);

	/* Use address of font handle */
	glcd_setfont(&FreeSerif12pt7b);


	int16_t xleft, yupper, w, h;

	glcd_plotstringwithfont(0, 150, "This is a serif font", GLCD_COLOR_PINK, GLCD_STRING_NORMAL);
	glcd_delay_ms(2000);
	glcd_plotstring(0, 10, "Find bounding box for string...", GLCD_COLOR_PINK, GLCD_COLOR_PINK, GLCD_STRING_NORMAL);
	glcd_getstringsizewithfont(0, 150, "This is a serif font", &xleft, &yupper, &w, &h, GLCD_STRING_NORMAL);
	glcd_plotrect(xleft, yupper, w, h, GLCD_COLOR_ORANGE);

	glcd_setfont(NULL);

	glcd_delay_ms(5000);
#endif

	/* Console based printing */
	glcd_puts("\fConsole based printing");
    for (i=0; i<10; i++) {
    	glcd_puts("Hello\n");
    }
	START_CLOCK();
	for (i=0; i<10; i++) {
		glcd_puts("12345678901234567890123456789012345678901234567890ABC\r");
	}
    GET_CLOCK(12);
    for (int i=0; i<10; i++) {
    	glcd_puts("Hello\n");
    }
	glcd_puts("123456789012345678901234567890123456789012345678901234567890\n");

	START_CLOCK();
    glcd_puts("\nConsole based print routines\n\nAnd another line\nAnd another one\b\b\bline\n\b\b\bhello\n");
    GET_CLOCK(13);

	glcd_delay_ms(3000);

	glcd_puts("\f");
	/* Plot all the grey scales */
	for (i=0; i<256; i++) {
		glcd_plotverticalline(i, 0, 240, (i<<16)+(i<<8)+i);
	}
	glcd_puts("Plotting grey scales");

	glcd_delay_ms(2000);

#ifdef HAVE_DOG
	/* Plot the image */
	START_CLOCK();
	glcd_plotbitmap8bpp(0, 0, 320, 240, dog_map, NULL);
    GET_CLOCK(14);
#endif

	glcd_delay_ms(2000);

	/* Plot the THUAS bitmap */
	glcd_cls(GLCD_COLOR_WHITE);
	START_CLOCK();
#ifdef GLCD_HAVE_THUAS_BITMAPS
	glcd_plotbitmap(0, 72, glcd_thuas_map, 320, 96, GLCD_COLOR_THUASGREEN, GLCD_COLOR_WHITE);
#else
	glcd_plotstring(100, 120, "Plotting THUAS bitmap disabled!", GLCD_COLOR_THUASGREEN, GLCD_COLOR_WHITE, GLCD_STRING_NORMAL);
#endif
	GET_CLOCK(15);
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

	/* Measurement of 100 ms */
    START_CLOCK();
    glcd_delay_ms(100);
    GET_CLOCK(19);

    PRINT_CLOCK();

#ifdef MEASUREMENT
	glcd_putchar('\n');
    for (i=9; i>0; i--) {
    	glcd_putchar(i+0x30);
    	glcd_puts(" seconds left\r");
    	glcd_delay_ms(1000);
    }
#else
	glcd_delay_ms(10000);
#endif
}

/* Rotation demo and the use of touchscreen */
void demo_rotation(void) {

	uint16_t p, raw;
	int16_t x, y;

	/* Rotation 90 degrees */
	glcd_cls(GLCD_COLOR_BLACK);

	glcd_setrotation(GLCD_DISPLAY_ROT90);

	glcd_plotrectfill(20, 120, 20, 20, GLCD_COLOR_YELLOW);
	glcd_plotstring(50, 126, "Touch rectangle", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	glcd_plotrectfill(20, 150, 20, 20, GLCD_COLOR_YELLOW);
	glcd_plotstring(50, 156, "Touch rectangle", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	glcd_plotrectfill(20, 180, 20, 20, GLCD_COLOR_YELLOW);
	glcd_plotstring(50, 186, "Touch for next", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	/* Get a coordinate from the touchscreen */
	while (1) {

		raw = touchscreen_readrawx();
		y = touchscreen_map(raw, TOUCH_LEFT, TOUCH_RIGHT, 0, glcd_getheight());

		raw = touchscreen_readrawy();
		x = glcd_getwidth() - touchscreen_map(raw, TOUCH_BOTTOM, TOUCH_TOP, 0, glcd_getwidth());

		p = touchscreen_pressure();

		if (touchscreen_ispressed(p)) {
			if (x>=20 && x<=20+20 && y>=120 && y<=120+20) {
				glcd_plotstring(20, 20, "Touched!", GLCD_COLOR_RED, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
			}
			if (x>=20 && x<=20+20 && y>=150 && y<=150+20) {
				glcd_plotstring(20, 20, "Touched!", GLCD_COLOR_BLUE, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
			}
			if (x>=20 && x<=20+20 && y>=180 && y<=180+20) {
				glcd_setrotation(GLCD_DISPLAY_ROT0);
				break;
			}
		} else {
			glcd_plotstring(20, 20, "Touched!", GLCD_COLOR_BLACK, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
		}
	}

	/* Rotation 180 degrees */
	glcd_cls(GLCD_COLOR_BLACK);

	glcd_setrotation(GLCD_DISPLAY_ROT180);

	glcd_plotrectfill(20, 120, 20, 20, GLCD_COLOR_YELLOW);
	glcd_plotstring(50, 126, "Touch rectangle", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	glcd_plotrectfill(20, 150, 20, 20, GLCD_COLOR_YELLOW);
	glcd_plotstring(50, 156, "Touch rectangle", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	glcd_plotrectfill(20, 180, 20, 20, GLCD_COLOR_YELLOW);
	glcd_plotstring(50, 186, "Touch for next", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	/* Get a coordinate from the touchscreen */
	while (1) {

		raw = touchscreen_readrawx();
		x = glcd_getwidth() - touchscreen_map(raw, TOUCH_LEFT, TOUCH_RIGHT, 0, glcd_getwidth());

		raw = touchscreen_readrawy();
		y = glcd_getheight() - touchscreen_map(raw, TOUCH_BOTTOM, TOUCH_TOP, 0, glcd_getheight());

		p = touchscreen_pressure();

		if (touchscreen_ispressed(p)) {
			if (x>=20 && x<=20+20 && y>=120 && y<=120+20) {
				glcd_plotstring(20, 20, "Touched!", GLCD_COLOR_RED, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
			}
			if (x>=20 && x<=20+20 && y>=150 && y<=150+20) {
				glcd_plotstring(20, 20, "Touched!", GLCD_COLOR_BLUE, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
			}
			if (x>=20 && x<=20+20 && y>=180 && y<=180+20) {
				glcd_setrotation(GLCD_DISPLAY_ROT0);
				break;
			}
		} else {
			glcd_plotstring(20, 20, "Touched!", GLCD_COLOR_BLACK, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
		}
	}

	/* Rotation 270 degrees */
	glcd_cls(GLCD_COLOR_BLACK);

	glcd_setrotation(GLCD_DISPLAY_ROT270);

	glcd_plotrectfill(20, 120, 20, 20, GLCD_COLOR_YELLOW);
	glcd_plotstring(50, 126, "Touch rectangle", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	glcd_plotrectfill(20, 150, 20, 20, GLCD_COLOR_YELLOW);
	glcd_plotstring(50, 156, "Touch rectangle", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	glcd_plotrectfill(20, 180, 20, 20, GLCD_COLOR_YELLOW);
	glcd_plotstring(50, 186, "Touch for exit", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	/* Get a coordinate from the touchscreen */
	while (1) {

		raw = touchscreen_readrawx();
		y = glcd_getheight() - touchscreen_map(raw, TOUCH_LEFT, TOUCH_RIGHT, 0, glcd_getheight());

		raw = touchscreen_readrawy();
		x = touchscreen_map(raw, TOUCH_BOTTOM, TOUCH_TOP, 0, glcd_getwidth());

		p = touchscreen_pressure();

		if (touchscreen_ispressed(p)) {
			if (x>=20 && x<=20+20 && y>=120 && y<=120+20) {
				glcd_plotstring(20, 20, "Touched!", GLCD_COLOR_RED, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
			}
			if (x>=20 && x<=20+20 && y>=150 && y<=150+20) {
				glcd_plotstring(20, 20, "Touched!", GLCD_COLOR_BLUE, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
			}
			if (x>=20 && x<=20+20 && y>=180 && y<=180+20) {
				glcd_setrotation(GLCD_DISPLAY_ROT0);
				break;
			}
		} else {
			glcd_plotstring(20, 20, "Touched!", GLCD_COLOR_BLACK, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
		}
	}
}

/* plot the character set and wait for touch */
void demo_characterset(void) {

	uint32_t i, j;
	char buffer[40];

	glcd_cls(GLCD_COLOR_BLACK);

	//glcd_setcharfont(0, 255, 1, 128, 0, 255);

	/* Plot the complete character table */
	for (j=0; j<16; j++) {
		snprintf(buffer, sizeof buffer, "%1lx", j);
		glcd_plotstring(40+j*10, 20, buffer, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
	}

	for (i=0; i<16; i++) {
		snprintf(buffer, sizeof buffer, "%1lx", i);
		glcd_plotstring(0, 40+i*10, buffer, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);
		for (j=0; j<16; j++) {
			glcd_plotchar(40+j*10, 40+i*10, i*16+j, GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK);
		}
	}

	glcd_plotstring(40, 220, "Touch the screen to exit", GLCD_COLOR_YELLOW, GLCD_COLOR_BLACK, GLCD_STRING_NORMAL);

	/* Wait for the screen to be (re)touched */
	while (touchscreen_ispressed(touchscreen_pressure())) {}
	while (!touchscreen_ispressed(touchscreen_pressure())) {}
}


void test(void) {

	uint32_t p;

	/* Now for some alternative fonts */
	glcd_cls(GLCD_COLOR_MAROON);

//	glcd_plotstring(0, 0, "Now some alternative fonts...", GLCD_COLOR_PINK, GLCD_COLOR_PINK, GLCD_STRING_CONDENSED);
//
//	/* Use address of font handle */
//	glcd_setfont(&FreeMono12pt7b);
//
//	glcd_plotstringwithfont(0, 100, "This is a monotype font", GLCD_COLOR_PINK, GLCD_STRING_CONDENSED);
//
//	/* Use address of font handle */
//	glcd_setfont(&FreeSerif12pt7b);
//
//	glcd_plotstringwithfont(0, 150, "a", GLCD_COLOR_PINK, GLCD_STRING_NORMAL);
//
//	uint16_t w, h;
//
//	glcd_getstringsizewithfont("a", &w, &h, GLCD_STRING_NORMAL);
//
//	glcd_plotrect(0, 150-h, w, h, GLCD_COLOR_YELLOW);

//	glcd_setfont(&FreeSerif12pt7b);

	//	int16_t x=0,y=100,minx=glcd_getwidth(),miny=glcd_getheight(),maxx=0,maxy=0;
//	glcd_plotstringwithfont(0, 100, "q", GLCD_COLOR_PINK, GLCD_STRING_NORMAL);
//	glcd_getcharboundshelper('q', &x, &y,
//	                              &minx, &miny, &maxx,
//	                              &maxy);
	//glcd_plotrect(minx, miny, maxx-minx, maxy-miny, GLCD_COLOR_YELLOW);
//
//	glcd_plotstring(10, 10, "Now some alternative fonts...", GLCD_COLOR_PINK, GLCD_COLOR_PINK, GLCD_STRING_NORMAL);
//	glcd_plotcircle(10, 10, 5, GLCD_COLOR_BLUE);
//
//	int16_t x=30, y=100, xleft, yupper;
//	int16_t w, h;
//
//	glcd_getstringsizewithfont(x, y, "HalloeQxqpjKklmnoy", &xleft, &yupper, &w, &h, GLCD_STRING_NORMAL);
////	glcd_plotrect(xleft, yupper, w, h, GLCD_COLOR_YELLOW);
//	glcd_plotrect(xleft-1, yupper-1, w+2, h+2, GLCD_COLOR_YELLOW);
//	glcd_plotstringwithfont(x, y, "HalloeQxqpjKklmnoy", GLCD_COLOR_PINK, GLCD_STRING_NORMAL);
//
//
//
//	glcd_plotcircle(xleft, yupper, 5, GLCD_COLOR_ORANGE);
//	glcd_plotcircle(x, y, 5, GLCD_COLOR_BLUE);

//	x = 100; y = 150;
//
//	glcd_plotcharwithfont(x, y, 'M', GLCD_COLOR_PINK);
//	glcd_getcharsizewithfont(x, y, 'M', &xleft, &yupper, &w, &h);
//	glcd_plotrect(xleft, yupper, w, h, GLCD_COLOR_YELLOW);

//	glcd_setfont(NULL);

	//glcd_plotcirclefill(20, 100, 30, GLCD_COLOR_PINK);
	//glcd_plotregularpolygonfill(100, 100, 130, 6, 0, GLCD_COLOR_PINK);
	//glcd_plotarc(50, 100, 80, -90.0, 90.0, GLCD_COLOR_PINK);

	//glcd_plotrect(120, 80, 300, 70, GLCD_COLOR_PINK);

//	glcd_plothorizontalline(360, 10, -100, GLCD_COLOR_PINK);
//	glcd_plotcircle(360-100, 10, 5, GLCD_COLOR_ORANGE);

	//glcd_plotverticalline(20, 20, 400, GLCD_COLOR_PINK);

	glcd_plotrectfill(100, 100, 300, 100, GLCD_COLOR_PINK);
	glcd_plotrectroundedfill(100, 100, 300, 100, 15, GLCD_COLOR_BLACK);
	glcd_plotrectrounded(100, 100, 300, 100, 15, GLCD_COLOR_ORANGE);

	glcd_delay_ms(100);

	while (1) {
		p = touchscreen_pressure();
		if (touchscreen_ispressed(p)) {
			break;
		}
	}


}

