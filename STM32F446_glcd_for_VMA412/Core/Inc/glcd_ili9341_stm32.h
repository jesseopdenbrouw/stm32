/*
 * Routines for using the VMA412 touchscreen Graphic LCD
 *
 * Note: pin assignment for STM32F446 Nucleo Board
 *
 * Note: the 8-bit 8080 CPU interface is used.

Software License Agreement (BSD License)

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

#ifndef INC_GLCD_ILI9341_STM32_H_
#define INC_GLCD_ILI9341_STM32_H_

#include <stdint.h>

/* How a string is spaced on the GLCD */
typedef enum {GLCD_STRING_CONDENSED=1, GLCD_STRING_NORMAL=4, GLCD_STRING_WIDE=8} glcd_spacing_t;

/* To invert the display */
typedef enum {GLCD_DISPLAY_INVERSION_OFF=0, GLCD_DISPLAY_INVERSION_ON=1} glcd_display_inversion_t;

/* To enter or leave idle mode */
typedef enum {GLCD_DISPLAY_IDLE_OFF=0, GLCD_DISPLAY_IDLE_ON=1} glcd_display_idle_t;

/* To turn the display on or off */
typedef enum {GLCD_DISPLAY_OFF=0, GLCD_DISPLAY_ON=1} glcd_display_t;

/* Some primary colors */
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
void glcd_read_terminate(uint16_t cmd, uint16_t amount, uint16_t data[]);
/* A write not explicitly terminated */
void glcd_write(uint16_t cmd, uint16_t amount, const uint16_t data[]);
/* Terminate a write */
void gcld_terminate_write(void);
/* Delay in ms */
void glcd_delay_ms(uint32_t delay);

/* High level commands */
/* Clear the screen */
void glcd_cls(uint32_t color);
/* Plot a pixel on (x,y) */
void glcd_plotpixel(uint16_t x, uint16_t y, uint32_t color);
/* Read pixel data */
uint32_t glcd_readpixel(uint16_t x, uint16_t y);
/* Plot a horizontal line starting on (x,y) with width w */
void glcd_plothorizontalline(uint16_t x, uint16_t y, uint16_t w, uint32_t color);
/* Plot a vertical line starting on (x,y) with height h */
void glcd_plotverticalline(uint16_t x, uint16_t y, uint16_t h, uint32_t color);
/* Plots a line from (x0,y0) to (x1,y1) */
void glcd_plotline(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t color);

/* Plot a character using the standard font */
void glcd_plotchar(uint16_t x, uint16_t y, uint8_t c, uint32_t color, uint32_t bg);
/* Plot a string using the standard font. Special characters are rendered using the font
 * and are not processed special. A \0 terminates the string. */
void glcd_plotstring(uint16_t x, uint16_t y, char str[], uint32_t color, uint32_t bg, glcd_spacing_t spacing);

/* Plot shapes */
void glcd_plotrect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t color);
void glcd_plotrectfill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t color);
void glcd_plotcircle(uint16_t x0, uint16_t y0, uint16_t r, uint32_t color);

/* Plot a two-color bitmap on the screen */
void glcd_plotbitmap(uint16_t x, uint16_t y, const uint8_t bitmap[], uint16_t w, uint16_t h, uint32_t color, uint32_t bg);

/* Turn display inversion on or off */
void glcd_inversion(glcd_display_inversion_t what);

/* Turn idle mode on or off */
void glcd_idle(glcd_display_idle_t what);

/* Turn display on or off */
void glcd_display(glcd_display_t what);

/* Flood fill an object */
//void glcd_floodfill(uint16_t x,uint16_t y, uint32_t fillColor, uint32_t defaultColor);

#endif /* INC_GLCD_ILI9341_STM32_H_ */
