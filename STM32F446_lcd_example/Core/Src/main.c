/*
 * STM32F4xx Nucleo Board LCD routines example source
 *
 * !! Please read the documentation below !!
 *
 * Version: 1.0
 * Date: 2020/02/29
 *
 * (c)2020, J.E.J. op den Brouw <J.E.J.opdenBrouw@hhs.nl>
 *
 * The Hague University of Applied Sciences (THUAS)
 * Dept. of Electrical Engineering
 *
 * This set of functions can be used with the Velleman VMA203
 * 16x2 LCD with HD44780 controller on a STM32F4xx Nucleo Board.
 * The VMA203 is an Arduino pin compatible display. The R/W-line
 * is tied to gnd, so it is not possible to read from the display,
 * e.g., to monitor the busy flag. The table below shows the pin
 * connections to the STM32F4xx Nucleo Board.
 *
 * See https://www.velleman.eu/products/view?id=435510&country=us&lang=nl
 *
 * Ard.  LCD    Nucleo
 * -------------------
 * D10   BL     PB6    !!! READ BELOW !!!
 * D9    E      PC7
 * D8    RS     PA9
 * D7    DB7    PA8
 * D6    DB6    PB10
 * D5    DB5    PB4
 * D4    DB4    PB5
 *
 * Tested with STM32F446 Nucleo Board and STM32CubeICE v 1.3.0
 * with clock frequency 168 MHz, optimizer -O0 and -O2
 *
 * The backlight is connected to pin PB6 and PB6 should only be used
 * as open drain output. Consult the schematics of the VMA203.
 * Never put a logic '1' on PB6!
 *
 * The VMA203 houses five push buttons connected to Analog pin A0.
 * Each button creates a analog voltage on Analog pin A0.
 * Please note: the Analog pin A0 is connected through a resistor to
 * the 5 V power supply. If no button is presses, this 5 V is connected
 * to Analog pin A0 and exceeds the maximum voltage of 3.3 V. This can
 * damage the Analog pin A0.
 *
 * Please connect a 5k6 resistor from Analog pin A0 to GND. This will
 * lower the voltage on Analog pin A0.
 *
 */

/* Set clock to 168 MHz */
#define USEMAXMHZ

#include <stdio.h>
#include <stdint.h>

#include "stm32f4xx.h"
#include "lcd.h"

/* Test for boards */
#if defined(STM32F446xx)
#define GLCD_RCC_M (378)
#elif defined(STM32F411xx)
#define GLCD_RCC_M (225)
#else
#undef USEMAXMHZ
#define GLCD_RCC_M (50)
#endif

int main(void) {

	int i;
	/* volatile for use with optimizer */
	volatile int dly;
	char buf[20];
	uint8_t chdef[] = {0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55};

	SystemInit();

#ifdef USEMAXMHZ

	/* 5 wait states */
	FLASH->ACR = (FLASH_ACR_LATENCY_5WS << FLASH_ACR_LATENCY_Pos);

	/* Caches enable, prefetch enable */
	FLASH->ACR |= (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN);

	RCC->CR |= RCC_CR_HSEBYP;

	/* Start the Main PLL to 168 MHz
	 * FREQ = ((HSE/M)*N)/P
	 * 1 MHz <= HSE/M <= 2 MHz
	 * 50 <= N <= 432      2 <= M <= 63    P = 2,4,6,8
	 */
	RCC->PLLCFGR = (GLCD_RCC_M<<RCC_PLLCFGR_PLLN_Pos) | (6<<RCC_PLLCFGR_PLLM_Pos) |
			       (2<<RCC_PLLCFGR_PLLP_Pos) | (0<<RCC_PLLCFGR_PLLSRC_Pos);

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

	/* Get the CPU speed from the I/O registers */
	/* This function sets SystemCoreClock to 100000000 */
	SystemCoreClockUpdate();

	lcd_init();
	lcd_init_backlight();

	lcd_set_cgram(0x00, chdef);

	lcd_putc('H');
	lcd_putc('e');
	lcd_putc('l');
	lcd_putc('l');
	lcd_putc('o');

	lcd_puts(" World! ");
	lcd_goto(1,0);
	lcd_puts("Char at pos 0: ");
	lcd_putc(0x00);

	for (dly=0; dly<3000000; dly++);
	lcd_cls();

	lcd_set_backlight(255/3);

	for (dly=0; dly<3000000; dly++);
	lcd_cursor(0, 0);

	for (dly=0; dly<3000000; dly++);
	lcd_home();
	lcd_puts("Booting.");
	for (i=0; i<6; i++) {
		for (dly=0; dly<300000; dly++);
		lcd_putc('.');
	}

	lcd_goto(1,6);
	lcd_putc('X');

	lcd_set_backlight(2*255/3);

	for (dly=0; dly<3000000; dly++);
	lcd_cls();
	lcd_goto(1,0);
	lcd_puts("From 0 to 32767");
	lcd_home();

	for (i=0; i<=32767; i++) {
		snprintf(buf, sizeof buf, "i = %5d", i);
		lcd_goto(0,0);
		lcd_puts(buf);
		lcd_set_backlight((uint8_t) (i>>2));
	}

	i = 0;
	while (1) {
		lcd_goto(1,0);
		lcd_puts("Now reset me!   ");
		i=(i+1)&0x7fff;
		lcd_set_backlight(i>>2);
	}

	return 0;
}
