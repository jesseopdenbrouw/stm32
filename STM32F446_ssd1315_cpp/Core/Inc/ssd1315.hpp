/*
 * ssd1315.hpp
 *
 *  Created on: Jan 20, 2024
 *      Author: jesse
 */

#ifndef INC_SSD1315_HPP_
#define INC_SSD1315_HPP_

#include "main.h"

typedef enum {SSD1315_OK, SSD1315_ERR} ssd1315_status_t;

#ifndef SSD1315_ADDR
#define SSD1315_ADDR (0x3C << 1)
#endif

/* Taken from the AdaFruit library */
#define ssd1315_swap_uint8_t(a, b)   \
	{                                \
		uint8_t t = a;               \
		a = b;                       \
		b = t;                       \
	}

class ssd1315 {

	public:
		ssd1315(I2C_HandleTypeDef *hi2c, uint16_t address) {
			phi2c = hi2c;
			addr = address;
		}
		ssd1315_status_t init(void);
		ssd1315_status_t plotpixel(uint8_t x, uint8_t y, uint8_t color);
		ssd1315_status_t updatescreen(void);
		ssd1315_status_t putchar(char ch);
		ssd1315_status_t puts(char *s);
		ssd1315_status_t setpos(uint8_t x, uint8_t y);
		void fillscreen(uint8_t val);
		void plotline(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);
		void plotcircle(int16_t x0, int16_t y0, int16_t r, uint8_t color);
		void plotrect(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);
		ssd1315_status_t setcontrast(uint8_t contrast);


	private:
		I2C_HandleTypeDef *phi2c = NULL;
		uint16_t addr = 0x00;
		uint8_t buffer[1025] = { 0x40, 0 };
		uint16_t write_pointer = 0;
};



#endif /* INC_SSD1315_HPP_ */
