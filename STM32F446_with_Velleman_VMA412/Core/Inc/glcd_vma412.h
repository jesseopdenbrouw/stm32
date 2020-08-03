/*
 * Routines for using the Velleman VMA412 touchscreen Graphic LCD
 *
 * Note: pin assignment for STM32F446 Nucleo Board
 *
 * Note: the 8-bit 8080 CPU interface is used.

Software License Agreement (BSD License)

Version: 0.1rc5
Date: 2020/08/03

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

#ifndef INC_GLCD_VMA412_H_
#define INC_GLCD_VMA412_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* Driver version */
#define GLCD_VERSION "STM32 VMA412 GLCD Driver v 0.1rc5 (Aug 3 2020)"


/* From here on, user can change settings */


/* Should we use flood fill? */
#define GLCD_USE_FLOOD_FILL
/* Increase if you have problems */
#define GLCD_STACK_SIZE (2000)
/* Print error message if stack overflows */
#define GLCD_USE_FLOOD_FILL_PRINT_IF_STACK_OVERFLOW
/* Do we include the arc function, needs sinf and cosf functions */
#define GLCD_USE_ARC
/* Do we include the regular polygon functions, needs sinf and cosf functions */
#define GLCD_USE_REGULAR_POLYGON
/* Character table in ROM or RAM */
/* Note: although you can set the character table to RAM, it
 * still occupies ROM. The character table is then copied
 * from ROM to RAM, when the STM32 is started.
 */
#define GLCD_CHARCTERS_IN_RAM
/* Define this to make use of the buildin THUAS bitmaps */
#define GLCD_HAVE_THUAS_BITMAPS
/* Do we need alternative fonts? */
#define GLCD_USE_ALTERNATIVE_FONTS


/* From here on, don't change anything */

/* Display idth and the height in pixels, landscape, do not change */
#define GLCD_WIDTH (320)
#define GLCD_HEIGHT (240)

#include <stdint.h>

/* Color is defined as unsigned 32 bit. DO NOT CHANGE */
typedef uint32_t glcd_color_t;

/* Buffer for actions is one of uint8_t or uint16_t */
/* For minimum resources, select uint8_t, but is slower */
/* For fastest code with some RAM slack, select uint16_t */
/* You could select uint32_t, but that's waste of space */
typedef uint16_t glcd_buffer_t;

/* For high level commands */
/* How to rotate the screen, by hardware */
typedef enum {GLCD_DISPLAY_ROT0=0, GLCD_DISPLAY_ROT90=1, GLCD_DISPLAY_ROT180=2, GLCD_DISPLAY_ROT270=3} glcd_rotation_t;

/* How a string is spaced on the GLCD */
typedef enum {GLCD_STRING_CONDENSED=1, GLCD_STRING_NORMAL=4, GLCD_STRING_WIDE=8} glcd_spacing_t;

/* To invert the display */
typedef enum {GLCD_DISPLAY_INVERSION_OFF=0, GLCD_DISPLAY_INVERSION_ON=1} glcd_display_inversion_t;

/* To enter or leave idle mode */
typedef enum {GLCD_DISPLAY_IDLE_OFF=0, GLCD_DISPLAY_IDLE_ON=1} glcd_display_idle_t;

/* To turn the display on or off */
typedef enum {GLCD_DISPLAY_OFF=0, GLCD_DISPLAY_ON=1} glcd_display_onoff_t;

/* To draw quarter circles */
typedef enum {GLCD_CORNER_UPPER_LEFT=1, GLCD_CORNER_UPPER_RIGHT=2, GLCD_CORNER_LOWER_RIGHT=4, GLCD_CORNER_LOWER_LEFT=8} glcd_corner_t;

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

/* Some secondary colors */
#define GLCD_COLOR_NAVY        (0x000080)
#define GLCD_COLOR_DARKGREEN   (0x008000)
#define GLCD_COLOR_DARKCYAN    (0x008080)
#define GLCD_COLOR_MAROON      (0x800000)
#define GLCD_COLOR_PURPLE      (0x800080)
#define GLCD_COLOR_OLIVE       (0x808000)
#define GLCD_COLOR_LIGHTGREY   (0xC0C0C0)
#define GLCD_COLOR_ORANGE      (0xFFA500)
#define GLCD_COLOR_GREENYELLOW (0xADFF2F)
#define GLCD_COLOR_PINK        (0xFFC0CB)

/* Gray 50% */
#define GLCD_COLOR_GREY50  (0x808080)

/* THUAS default color */
#define GLCD_COLOR_THUASGREEN ((158<<16)|(167<<8)|0)

/* Taken from the AdaFruit library */
#define glcd_swap_uint16_t(a, b)                                                   \
	{                                                                              \
		uint16_t t = a;                                                            \
		a = b;                                                                     \
		b = t;                                                                     \
	}

#ifdef GLCD_USE_ALTERNATIVE_FONTS
/* From the Adafruit library, for using custom fonts */
// Font data stored PER GLYPH
typedef struct GFXglyph_struct{
  uint16_t bitmapOffset; ///< Pointer into GFXfont->bitmap
  uint8_t width;         ///< Bitmap dimensions in pixels
  uint8_t height;        ///< Bitmap dimensions in pixels
  uint8_t xAdvance;      ///< Distance to advance cursor (x axis)
  int8_t xOffset;        ///< X dist from cursor pos to UL corner
  int8_t yOffset;        ///< Y dist from cursor pos to UL corner
} GFXglyph;

// Data stored for FONT AS A WHOLE
typedef struct GFXfont_struct {
  uint8_t *bitmap;  ///< Glyph bitmaps, concatenated
  GFXglyph *glyph;  ///< Glyph array
  uint16_t first;   ///< ASCII extents (first char)
  uint16_t last;    ///< ASCII extents (last char)
  uint8_t yAdvance; ///< Newline distance (y axis)
} GFXfont;
#endif

/* Low level commands
 * These command are normally not needed by the user
 */


/* A read is always explicitly terminated */
void glcd_read_terminate(uint16_t cmd, uint16_t amount, glcd_buffer_t data[]);
/* A write not explicitly terminated */
void glcd_write(uint16_t cmd, uint16_t amount, const glcd_buffer_t data[]);
/* Terminate a write */
void glcd_terminate_write(void);
/* Sets the delay (width) of the read pulse, USE WITH CARE */
void glcd_set_read_pulse_delay(uint32_t delay);
/* Sets the delay (width) of the write pulse, USE WITH CARE */
void glcd_set_write_pulse_delay(uint32_t delay);
/* set the delay (read) of the read pulse, USE WITH CARE */


/* High level commands
 * Used by the application
 */


/* Basic functions */
/* Initialize display */
/* MUST BE CALLED *AFTER* THE CLOCK SYSTEM IS SETUP */
void glcd_init(void);
/* Delay in ms */
void glcd_delay_ms(glcd_color_t delay);
/* Set screen rotation */
void glcd_setrotation(glcd_rotation_t rot);
/* Plot a pixel on (x,y) */
void glcd_plotpixel(int16_t x, int16_t y, glcd_color_t color);
/* Read pixel data */
glcd_color_t glcd_readpixel(int16_t x, int16_t y);


/*
 *  Plotting and printing characters
 *
 */

/* Plot a character using the standard font */
void glcd_plotchar(int16_t x, int16_t y, uint8_t c, glcd_color_t color, glcd_color_t bg);
/* Plot a string using the standard font. Special characters are rendered using the font
 * and are not processed special. A \0 terminates the string. */
void glcd_plotstring(int16_t x, int16_t y, char str[], glcd_color_t color, glcd_color_t bg, glcd_spacing_t spacing);
/* Set the x and y magnification for standard font */
void glcd_setcharsize(int16_t sizx, int16_t sizy);
/* Set the font layout of a character of the standard font. Font must be in RAM. */
void glcd_setcharlayout(uint16_t c, uint16_t byte0, uint16_t byte1, uint16_t byte2, uint16_t byte3, uint16_t byte4);
/*
 * Using an alternate font from AdaFruit
 */
/* Set alternate font */
void glcd_setfont(const GFXfont *f);
/* Plot character using alternate font */
void glcd_plotcharwithfont(int16_t x, int16_t y, uint8_t c, glcd_color_t color);
/* Plot string using alternate font */
void glcd_plotstringwithfont(int16_t x, int16_t y, char str[], glcd_color_t color, glcd_spacing_t spacing);
/* Get the upper-left corner, width and height of a character using alternate font */
void glcd_getcharsizewithfont(int16_t x, int16_t y, char c, int16_t *x1, int16_t *y1, int16_t *w, int16_t *h);
/* Get the upper-left corner, width and height of a string using alternate font */
void glcd_getstringsizewithfont(int16_t x, int16_t y, char *str, int16_t *x1, int16_t *y1, int16_t *w, int16_t *h, glcd_spacing_t);
/*
 *  Plot shapes
 *
 */

/* Clear the screen */
void glcd_cls(glcd_color_t color);
/* Plot a horizontal line starting on (x,y) with width w */
void glcd_plothorizontalline(int16_t x, int16_t y, int16_t w, glcd_color_t color);
/* Plot a vertical line starting on (x,y) with height h */
void glcd_plotverticalline(int16_t x, int16_t y, int16_t h, glcd_color_t color);
/* Plots a line from (x0,y0) to (x1,y1) */
void glcd_plotline(int16_t x0, int16_t y0, int16_t x1, int16_t y1, glcd_color_t color);
/* Plot a rectangle */
void glcd_plotrect(int16_t x, int16_t y, int16_t w, int16_t h, glcd_color_t color);
/* Plot a filled rectangle */
void glcd_plotrectfill(int16_t x, int16_t y, int16_t w, int16_t h, glcd_color_t color);
/* Plot a rectangle with rounded corners */
void glcd_plotrectrounded(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, glcd_color_t color);
/* Plot a filled rectangle with rounded corners */
void glcd_plotrectroundedfill(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, glcd_color_t color);
/* Plot a circle */
void glcd_plotcircle(int16_t xc, int16_t yc, int16_t r, glcd_color_t color);
/* Plot a filled circle */
void glcd_plotcirclefill(int16_t xc, int16_t yc, int16_t r, glcd_color_t color);
/* Helper functions, normally not needed */
void glcd_plotcirclequarter(int16_t x0, int16_t y0, int16_t r, glcd_corner_t cornername, glcd_color_t color);
void glcd_plotcirclehalffill(int16_t x0, int16_t y0, int16_t r, glcd_cornerhalves_t corners, int16_t delta, glcd_color_t color);
/* Plot an arc */
#ifdef GLCD_USE_ARC
void glcd_plotarc(int16_t xc, int16_t yc, int16_t r, float start, float stop, glcd_color_t color);
#endif
/* Plot a triangle */
void glcd_plottriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, glcd_color_t color);
/* Plot a filled rectangle */
void glcd_plottrianglefill(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, glcd_color_t color);
#ifdef GLCD_USE_REGULAR_POLYGON
/* Plot a regular polygon */
void glcd_plotregularpolygon(int16_t xc, int16_t yc, int16_t r, int16_t sides, float displ, glcd_color_t color);
/* Plot a filled regular polygon */
void glcd_plotregularpolygonfill(int16_t xc, int16_t yc, int16_t r, int16_t sides, float displ, glcd_color_t color);
#endif


/*
 * Plotting bitmaps
 */


/* Plot a two-color bitmap on the display with explicit colors */
void glcd_plotbitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, glcd_color_t color, glcd_color_t bg);
/* Plot a 256-color indexed bitmap on the display */
void glcd_plotbitmap8bpp(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pic, const uint8_t *palette);


/*
 * Miscellaneous
 */


/* Turn display inversion on or off */
void glcd_inversion(glcd_display_inversion_t what);
/* Turn idle mode on or off */
void glcd_idle(glcd_display_idle_t what);
/* Turn display on or off */
void glcd_display(glcd_display_onoff_t what);
/* Scroll the screen up vertical (software) */
void glcd_scrollvertical(int16_t lines);


/*
 * Flood fill objects
 *
 */


#ifdef GLCD_USE_FLOOD_FILL
/* Flood fill an object */
#ifndef GLCD_STACK_SIZE
#define GLCD_STACK_SIZE (2000)
#endif
void glcd_floodfill(int16_t xs, int16_t ys, glcd_color_t fillColor, glcd_color_t defaultColor);
#endif


/*
 * Simple console based printing
 *
 */


/* Printing character on console based output */
void glcd_putchar(char c);
/* Printing a string on console based output */
void glcd_puts(char str[]);


/*
 * Converting or getting information
 *
 */


/* Get the current width of the screen */
int16_t glcd_getwidth(void);

/* Get the current height of the screen */
int16_t glcd_getheight(void);

/* Convert 16 bit colors (5/6/5) to glcd_color_t */
glcd_color_t glcd_convertcolor(uint16_t color16);


#ifdef __cplusplus
}
#endif

#endif /* INC_GLCD_VMA412_H_ */
