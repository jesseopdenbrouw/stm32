/*
 * Routines for using the Velleman VMA412 touchscreen Graphic LCD
 *
 * Note: pin assignment for STM32F446 Nucleo Board
 *
 * Note: the 8-bit 8080 CPU interface is used.

Software License Agreement (BSD License)

Version: 0.1rc1
Date: 2020/07/19

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

#ifndef INC_GLCD_ILI9341_VMA412_H_
#define INC_GLCD_ILI9341_VMA412_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* Driver version */
#define GLCD_VERSION "STM32 VMA412 GLCD Driver v 0.1rc1 (Jul 19 2020)"


/* Should we use flood fill? */
#define GLCD_USE_FLOOD_FILL
/* Increase if you have problems */
#define GLCD_STACK_SIZE (2000)
/* Print error message if stack overflows */
#define GLCD_USE_FLOOD_FILL_PRINT_IF_STACK_OVERFLOW
/* Do we include the arc function, needs sinf and cosf functions */
#define GLCD_USE_ARC
/* Character table in ROM or RAM */
/* Note: although you can set the character table to RAM, it
 * still occupies ROM. The character table is then copied
 * from ROM to RAM, when the STM32 is started.
 */
//#define GLCD_CHARCTERS_IN_RAM
/* Define this to make use of the buildin THUAS bitmaps */
#define GLCD_HAVE_THUAS_BITMAPS

/* Display idth and the height in pixels, landscape, do not change */
#define GLCD_WIDTH (320)
#define GLCD_HEIGHT (240)

#include <stdint.h>

/* Color is defined as unsigned 32 bit. DO NOT CHANGE */
typedef uint32_t glcd_color_t;

/* Buffer for actions is one of uint8_t or uint16_t */
/* For minimum resources, select uint8_t */
/* For fastest code, select uint16_t */
/* You could select uint32_t, but that's waste of space and time */
typedef uint16_t glcd_buffer_t;

/* For high level commands */
/* How to rotate the screen, by hardware */
typedef enum {GLCD_SCREEN_ROT0=0, GLCD_SCREEN_ROT90=1, GLCD_SCREEN_ROT180=2, GLCD_SCREEN_ROT270=3} glcd_rotation_t;

/* How a string is spaced on the GLCD */
typedef enum {GLCD_STRING_CONDENSED=1, GLCD_STRING_NORMAL=4, GLCD_STRING_WIDE=8} glcd_spacing_t;

/* To invert the display */
typedef enum {GLCD_DISPLAY_INVERSION_OFF=0, GLCD_DISPLAY_INVERSION_ON=1} glcd_display_inversion_t;

/* To enter or leave idle mode */
typedef enum {GLCD_DISPLAY_IDLE_OFF=0, GLCD_DISPLAY_IDLE_ON=1} glcd_display_idle_t;

/* To turn the display on or off */
typedef enum {GLCD_DISPLAY_OFF=0, GLCD_DISPLAY_ON=1} glcd_display_t;

/* To draw quarter circles */
typedef enum {GLCD_CORNER_UPPER_LEFT=1, GLCD_CORNER_UPPER_RIGHT=2, GLCD_CORNER_LOWER_RIGHT=4, GLCD_CORNER_LOWER_LEFT=8} glcd_corner_t;
//typedef glcd_corner_t uint16_t;
//#define GLCD_CORNER_UPPER_LEFT 1
//#define GLCD_CORNER_UPPER_RIGHT 2
//#define GLCD_CORNER_LOWER_RIGHT 4
//#define GLCD_CORNER_LOWER_LEFT 8

/* To draw left, right of both corner halves */
typedef enum {GLCD_CORNER_LEFT_HALF=1, GLCD_CORNER_RIGHT_HALF=2, GLCD_CORNER_BOTH=3} glcd_cornerhalves_t;

/* For low level commands */
/* How to use the CS pin */
typedef enum {GLCD_CS_TO_HIGH, GLCD_CS_KEEP_LOW, GLCD_CS_TOGGLE} glcd_cs_t;
/* Direction of the data pins */
typedef enum {GLCD_KEEP_DATA_OUTPUT, GLCD_MAKE_DATA_INPUT} glcd_dir_t;

/* Some primary colors
 * Colors are made up by 00rrggbb
 * 00 = 0 (zero), must keep to 00
 * rr = red 8 bit, only upper 6 bits are use
 * gg = green 8 bit, only upper 6 bits are use
 * bb = blue 8 bit, only upper 6 bits are use
 */
#define GLCD_COLOR_BLACK   (0x000000)
#define GLCD_COLOR_BLUE    (0x0000ff)
#define GLCD_COLOR_GREEN   (0x00ff00)
#define GLCD_COLOR_CYAN    (0x00ffff)
#define GLCD_COLOR_RED     (0xff0000)
#define GLCD_COLOR_MAGENTA (0xff00ff)
#define GLCD_COLOR_YELLOW  (0xffff00)
#define GLCD_COLOR_WHITE   (0xffffff)

/* Gray 50% */
#define GLCD_COLOR_GREY50  (0x7f7f7f)

/* THUAS default color */
#define GLCD_COLOR_THUASGREEN ((158<<16)|(167<<8)|0)

/* THUAS internal present logo bitmaps */
#define GLCD_THUAS_DEFAULT_BITMAP (NULL)
#define GLCD_THUAS_DEFAULT_BITMAP_SMALL ((void *)-1) /* Hopes it works ;-) */

/* Taken from the AdaFruit library */
#define glcd_swap_uint16_t(a, b)                                                   \
	{                                                                              \
		uint16_t t = a;                                                            \
		a = b;                                                                     \
		b = t;                                                                     \
	}

/* Initialize display */
/* MUST BE CALLED *AFTER* THE CLOCK SYSTEM IS SETUP */
void glcd_init(void);

/* Low level commands */
/* A read is always explicitly terminated */
void glcd_read_terminate(uint16_t cmd, uint16_t amount, glcd_buffer_t data[]);
/* A write not explicitly terminated */
void glcd_write(uint16_t cmd, uint16_t amount, const glcd_buffer_t data[]);
/* Terminate a write */
void gcld_terminate_write(void);
/* Sets the delay (width) of the read pulse, USE WITH CARE */
void glcd_set_read_pulse_delay(uint32_t delay);
/* Sets the delay (width) of the write pulse, USE WITH CARE */
void glcd_set_write_pulse_delay(uint32_t delay);
/* set the delay (read) of the read pulse, USE WITH CARE */


/* High level commands */
/* Delay in ms */
void glcd_delay_ms(glcd_color_t delay);
/* Set screen rotation */
void glcd_setrotation(glcd_rotation_t rot);
/* Clear the screen */
void glcd_cls(uint32_t color);
/* Plot a pixel on (x,y) */
void glcd_plotpixel(uint16_t x, uint16_t y, glcd_color_t color);
/* Read pixel data */
glcd_color_t glcd_readpixel(uint16_t x, uint16_t y);
/* Plot a horizontal line starting on (x,y) with width w */
void glcd_plothorizontalline(uint16_t x, uint16_t y, uint16_t w, glcd_color_t color);
/* Plot a vertical line starting on (x,y) with height h */
void glcd_plotverticalline(uint16_t x, uint16_t y, uint16_t h, glcd_color_t color);
/* Plots a line from (x0,y0) to (x1,y1) */
void glcd_plotline(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, glcd_color_t color);

/* Plot a character using the standard font */
void glcd_plotchar(uint16_t x, uint16_t y, uint8_t c, glcd_color_t color, glcd_color_t bg);
/* Plot a string using the standard font. Special characters are rendered using the font
 * and are not processed special. A \0 terminates the string. */
void glcd_plotstring(uint16_t x, uint16_t y, char str[], glcd_color_t color, glcd_color_t bg, glcd_spacing_t spacing);

/* Plot shapes */
void glcd_plotrect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, glcd_color_t color);
void glcd_plotrectfill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, glcd_color_t color);
void glcd_plotrectrounded(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, glcd_color_t color);
void glcd_plotrectroundedfill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, glcd_color_t color);
void glcd_plotcircle(uint16_t x0, uint16_t y0, uint16_t r, glcd_color_t color);
void glcd_plotcirclefill(uint16_t x0, uint16_t y0, uint16_t r, glcd_color_t color);
void glcd_plotcirclequarter(uint16_t x0, uint16_t y0, uint16_t r, glcd_corner_t cornername, glcd_color_t color);
void glcd_plotcirclehalffill(uint16_t x0, uint16_t y0, uint16_t r, glcd_cornerhalves_t corners, int16_t delta, glcd_color_t color);
#ifdef GLCD_USE_ARC
void glcd_plotarc(uint16_t xc, uint16_t yc, uint16_t r, float start, float stop, glcd_color_t color);
#endif
void glcd_plottriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, glcd_color_t color);

/* Plot a two-color bitmap on the display */
void glcd_plotbitmap(uint16_t x, uint16_t y, const uint8_t bitmap[], uint16_t w, uint16_t h, glcd_color_t color, glcd_color_t bg);
/* Plot a 256-color indexed bitmap on the display */
void glcd_plotbitmap8bpp(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pic, const uint8_t *palette);

/* Turn display inversion on or off */
void glcd_inversion(glcd_display_inversion_t what);
/* Turn idle mode on or off */
void glcd_idle(glcd_display_idle_t what);
/* Turn display on or off */
void glcd_display(glcd_display_t what);

#ifdef GLCD_USE_FLOOD_FILL
/* Flood fill an object */
#ifndef GLCD_STACK_SIZE
#define GLCD_STACK_SIZE (2000)
#endif
void glcd_floodfill(uint16_t xs,uint16_t ys, glcd_color_t fillColor, glcd_color_t defaultColor);
#endif

/* Scroll the screen up vertical (software) */
void glcd_scrollvertical(uint16_t lines);

/* Printing character on console based output */
void glcd_putchar(char c);
/* Printing a string on console bases output */
void glcd_puts(char str[]);

/* Get the current width of the screen */
uint16_t glcd_getwidth(void);

/* Get the current height of the screen */
uint16_t glcd_getheight(void);

/* Convert 16 bit colors to glcd_color_t */
glcd_color_t glcd_convertcolor(uint16_t color16);

#ifdef __cplusplus
}
#endif

#endif /* INC_GLCD_ILI9341_VMA412_H_ */
