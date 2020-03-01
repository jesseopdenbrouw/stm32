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


#if !defined(STM32F446xx) && !defined(STM32F411xx)
#warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#warning ! Only tested with STM32F446 Nucleo Board !
#warning ! Only tested with STM32F411 Nucleo Board !
#warning !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#endif

#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"
#include "lcd.h"

void SystemCoreClockUpdate(void);

/* PWM repetition rate and timer prescaler */
#define LCD_REP_FREQ (600)
#define LCD_PRESCALER (20)
#define LCD_TIM_TOP ((uint32_t)(SystemCoreClock/(LCD_PRESCALER*LCD_REP_FREQ)))

/* Initialization struct for STM32 pinout */
typedef struct {
	GPIO_TypeDef *LCD_GPIO_E;
	uint32_t     LCD_pin_E;
	GPIO_TypeDef *LCD_GPIO_RS;
	uint32_t     LCD_pin_RS;
	GPIO_TypeDef *LCD_GPIO_RW;
	uint32_t     LCD_pin_RW;
	GPIO_TypeDef *LCD_GPIO_D7;
	uint32_t     LCD_pin_D7;
	GPIO_TypeDef *LCD_GPIO_D6;
	uint32_t     LCD_pin_D6;
	GPIO_TypeDef *LCD_GPIO_D5;
	uint32_t     LCD_pin_D5;
	GPIO_TypeDef *LCD_GPIO_D4;
	uint32_t     LCD_pin_D4;
} lcd_pins_t;

/* STM42F446 */
static const lcd_pins_t lcd = {
		GPIOC, 7,	/* E */
		GPIOA, 9,	/* RS */
		NULL,  0,	/* RW */
		GPIOA, 8,	/* D7 */
		GPIOB, 10,	/* D6 */
		GPIOB, 4,	/* D5 */
		GPIOB, 5,	/* D4 */
};

/* Should be automatically computed */
static const uint32_t lcd_ports_used = (1<<RCC_AHB1ENR_GPIOAEN_Pos)|(1<<RCC_AHB1ENR_GPIOBEN_Pos)|(1<<RCC_AHB1ENR_GPIOCEN_Pos);

/* Variable prems -- prescaler ms
 *          preus -- prescaler us
 * @private
 */
static uint32_t prems;
static uint32_t preus;

/* Function lcd_delay_init
 * Initializes the delay routine
 * @private
 * @in: void
 * @out: void
 */
static void lcd_delay_init(void) {
	SystemCoreClockUpdate();
	prems = SystemCoreClock/3000UL+1UL;
	preus = SystemCoreClock/3000000UL+1UL;
}

/* Function lcd_delay_ms
 * Creates a delay for the LCD functions
 * @private
 * @in: delay --> delay in millisecs
 * @out: void
 */
static void lcd_delay_ms(uint32_t delay) {

	uint32_t ms = prems*delay;

	asm volatile ("mov r3, %[ms] \n\t"
				  ".Llcd%=: \n\t"
				  "subs r3, #1 \n\t"
				  "bne .Llcd%= \n\t"
                  :
                  : [ms] "r" (ms)
                  : "r3", "cc"
				 );
}

/* Function lcd_delay
 * Creates a delay for the LCD functions
 * @private
 * @in: delay --> delay in microsecs
 * @out: void
 */
static void lcd_delay_us(uint32_t delay) {

	uint32_t us = preus*delay;

	asm volatile ("mov r3, %[us] \n\t"
				  ".Llcdu%=: \n\t"
				  "subs r3, #1 \n\t"
				  "bne .Llcdu%= \n\t"
                  :
                  : [us] "r" (us)
                  : "r3", "cc"
				 );
}

/* Function lcd_write_nibble
 * This function writes a nibble to the LCD. Only the
 * lower 4 bits of the data are written. One can issue
 * data or a command.
 * @private
 * @in: data --> the data to be written
 * @in: command --> command (1) or data (0)
 * @out: void
 */
static void lcd_write_nibble(uint8_t data, uint8_t command){

	/* Reset the data pins */
	lcd.LCD_GPIO_D7->BSRR = (1<<(lcd.LCD_pin_D7+16));
	lcd.LCD_GPIO_D6->BSRR = (1<<(lcd.LCD_pin_D6+16));
	lcd.LCD_GPIO_D5->BSRR = (1<<(lcd.LCD_pin_D5+16));
	lcd.LCD_GPIO_D4->BSRR = (1<<(lcd.LCD_pin_D4+16));

	/* Populate data pins */
	if (data&0x08) {
		lcd.LCD_GPIO_D7->BSRR = (1<<lcd.LCD_pin_D7);
	}
	if (data&0x04) {
		lcd.LCD_GPIO_D6->BSRR = (1<<lcd.LCD_pin_D6);
	}
	if (data&0x02) {
		lcd.LCD_GPIO_D5->BSRR = (1<<lcd.LCD_pin_D5);
	}
	if (data&0x01) {
		lcd.LCD_GPIO_D4->BSRR = (1<<lcd.LCD_pin_D4);
	}

	/* Command --> RS = 0, Data --> RS = 1 */
	if (command) {
		lcd.LCD_GPIO_RS->BSRR = (1<<(lcd.LCD_pin_RS+16));
	} else {
		lcd.LCD_GPIO_RS->BSRR = (1<<lcd.LCD_pin_RS);
	}

	/* Set EN high */
	lcd.LCD_GPIO_E->BSRR = (1<<lcd.LCD_pin_E);
	/* Wait 5 us */
	lcd_delay_us(5);
	/* Set EN low */
	lcd.LCD_GPIO_E->BSRR = (1<<(lcd.LCD_pin_E+16));
}

/* Function lcd_command
 * Writes a command to the lcd.
 * @public
 * @in: command --> the command to be written
 * @out: void
 */
static void lcd_command(uint8_t command) {
	lcd_write_nibble(command>>4, 1);
	lcd_write_nibble(command&0xf, 1);

	if (command<4) {
		lcd_delay_ms(2);
	} else {
		lcd_delay_ms(1);
	}
}

/* Function lcd_cursor
 * Sets the cursor characteristics
 * @public
 * @in: cursor --> on (1), off (0)
 * @in: blink --> blink (1), no blink (0)
 * @out: void
 */
void lcd_cursor(uint8_t cursor, uint8_t blink) {
	lcd_command(0x0c | ((cursor)?0x02:0x00) | ((blink)?0x01:0x00));
}

/* Function lcd_putc
 * Writes a character to the lcd to be displayed
 * @public
 * @in: data --> character to be displayed
 * @out: void
 */
void lcd_putc(uint8_t data) {
	lcd_write_nibble(data>>4, 0);
	lcd_write_nibble(data&0xf, 0);
	/* wait 50 us */
	lcd_delay_us(50);
}

/* Function lcd_puts
 * Writes a \0-terminated string to the display
 * @public
 * @in: str --> address of string in memory
 * @out: void
 */
void lcd_puts(char *str) {

	if (str == NULL) {
		return;
	}

	while (*str != '\0') {
		lcd_putc(*str);
		str++;
	}
}

/* Function lcd_cls
 * Clears the lcd screen and returns cursor to home
 * @public
 * @in: void
 * @out: void
 */
void lcd_cls(void) {
	lcd_command(0x01);
}

/* Function lcd_home
 * Returns cursor to home
 * @public
 * @in: void
 * @out: void
 */
void lcd_home(void) {
	lcd_command(0x02);
}

/* Function: lcd_goto
 * Puts cursor to line and column
 * @public
 * @in: row: one of {0,1,2,3}
 * @in: col: one of {0..19}
 * @out: void
 */
void lcd_goto(uint8_t row, uint8_t col) {
	uint8_t command = 0x80;

	switch (row) {
	case 1: command = 0xc0;
			break;
	case 2: command = 0x94;
			break;
	case 3: command = 0xd4;
			break;
	}

	lcd_command(command+col);
}

/* Function lcd_set_cgram
 * Writes a character definition into the character RAM
 * @public
 * @in: charno --> The character number (0..7)
 * @in: data[8] --> An 8-byte array with the definition
 * @out: void
 */
void lcd_set_cgram(uint8_t charno, uint8_t data[8]) {
	uint8_t i;

	lcd_command(0x40+(charno&0x07));
	for (i=0; i<8; i++) {
		lcd_putc(data[i]);
	}
	lcd_home();
}

/* Function lcd_init
 * Initializes the lcd
 * @public
 * @in: void
 * @out: void
 */
void lcd_init(void) {
	/* enable GPIOA/B/C clock */
    RCC->AHB1ENR |= lcd_ports_used;

    lcd.LCD_GPIO_E->MODER &= ~(3<<(lcd.LCD_pin_E*2));
    lcd.LCD_GPIO_E->MODER |= (1<<(lcd.LCD_pin_E*2));
    lcd.LCD_GPIO_E->BSRR = (1<<(lcd.LCD_pin_E+16));

    lcd.LCD_GPIO_RS->MODER &= ~(3<<(lcd.LCD_pin_RS*2));
    lcd.LCD_GPIO_RS->MODER |= (1<<(lcd.LCD_pin_RS*2));

    lcd.LCD_GPIO_D7->MODER &= ~(3<<(lcd.LCD_pin_D7*2));
    lcd.LCD_GPIO_D7->MODER |= (1<<(lcd.LCD_pin_D7*2));

    lcd.LCD_GPIO_D6->MODER &= ~(3<<(lcd.LCD_pin_D6*2));
    lcd.LCD_GPIO_D6->MODER |= (1<<(lcd.LCD_pin_D6*2));

    lcd.LCD_GPIO_D5->MODER &= ~(3<<(lcd.LCD_pin_D5*2));
    lcd.LCD_GPIO_D5->MODER |= (1<<(lcd.LCD_pin_D5*2));

    lcd.LCD_GPIO_D4->MODER &= ~(3<<(lcd.LCD_pin_D4*2));
    lcd.LCD_GPIO_D4->MODER |= (1<<(lcd.LCD_pin_D4*2));

	/* Initialize the delay routine */
	lcd_delay_init();

	/* Start lcd initialization routine */
	lcd_delay_ms(20);
	lcd_write_nibble(0x03, 1);
	lcd_delay_ms(5);
	lcd_write_nibble(0x03, 1);
	lcd_delay_ms(1);
	lcd_write_nibble(0x03, 1);
	lcd_delay_ms(1);

	/* Set 4-bit mode */
	lcd_write_nibble(0x02, 1);
	lcd_delay_ms(2);

	/* 4-bit mode, 2-line, 5x7 font */
	lcd_command(0x28);
	/* Move cursor right */
	lcd_command(0x06);
	/* Clear screen, cursor home */
	lcd_command(0x01);
	/* Turn on display, cursor on and blink */
	lcd_command(0x0f);

}

/* Function lcd_init_backlight
 * Initializes the lcd's backlight for use
 * with PWM
 * @public
 * @in: void
 * @out: void
 */
void lcd_init_backlight(void) {

	SystemCoreClockUpdate();                       /* Get the clock speed */

	RCC->AHB1ENR |= (1<<RCC_AHB1ENR_GPIOAEN_Pos);  /* enable GPIOA clock */

    GPIOB->AFR[0] |= (2<<GPIO_AFRL_AFSEL6_Pos);    /* PB6 pin for TIM2/CH1 (AF=2) */
    GPIOB->OTYPER |= (1<<GPIO_OTYPER_OT6_Pos);     /* Set open drain for PB6 */
    GPIOB->MODER &= ~(3<<GPIO_MODER_MODER6_Pos);   /* Clear MODER bits */
    GPIOB->MODER |=  (2<<GPIO_MODER_MODER6_Pos);   /* Select Alternate Function */

    /* setup TIM2 */
    RCC->APB1ENR |= (1<<RCC_APB1ENR_TIM4EN_Pos);   /* Enable TIM4 clock */
    TIM4->PSC = LCD_PRESCALER - 1;                 /* Divided by PRESCALER */
    TIM4->ARR = LCD_TIM_TOP - 1;                   /* Divided by TIM_TOP */
    TIM4->CNT = 0;                                 /* Set counter to 0 */
    TIM4->CCMR1 = 6<<TIM_CCMR1_OC1M_Pos;           /* PWM mode */
    TIM4->CCER = 1<<TIM_CCER_CC1E_Pos;             /* Enable PWM CH1 */
    TIM4->CCR1 = LCD_TIM_TOP - 1;                  /* Maximum brightness */
    TIM4->CR1 = 1<<TIM_CR1_CEN_Pos;                /* Enable timer */
}

/* Function lcd_set_backlight
 * Sets the lcd's backlight
 * with PWM
 * @public
 * @in: level --> 0 (off) to 255 (max)
 * @out: void
 */
void lcd_set_backlight(uint8_t level) {

	// Use some correction for the perception of the human eye.
	TIM4->CCR1 = LCD_TIM_TOP * level*level/255/255;
	//TIM4->CCR1 = LCD_TIM_TOP * level/255;
}
