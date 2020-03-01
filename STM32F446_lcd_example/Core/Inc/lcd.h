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

#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>
#include "stm32f4xx.h"

/* API */
void lcd_init(void);
void lcd_cursor(uint8_t cursor, uint8_t blink);
void lcd_cls(void);
void lcd_home(void);
void lcd_goto(uint8_t row, uint8_t col);
void lcd_putc(uint8_t data);
void lcd_puts(char *str);
void lcd_set_cgram(uint8_t charno, uint8_t data[8]);
void lcd_init_backlight(void);
void lcd_set_backlight(uint8_t level);

#endif /* LCD_H_ */
