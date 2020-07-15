/*
 * Routines for using the Velleman VMA412 touchscreen Graphic LCD
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

#ifndef INC_STM32F446_TOUCHSCREEN_H_
#define INC_STM32F446_TOUCHSCREEN_H_

#include "main.h"

/* Driver version */
#define TOUCH_VERSION "STM32 VMA412 TS Driver Version 0.1 (Jul 15 2020)"

/* Calibrate the touchscreen */
#define TOUCH_LEFT 120
#define TOUCH_RIGHT 940
#define TOUCH_TOP 900
#define TOUCH_BOTTOM 120
#define TOUCH_PRESSURE_LOW 300
#define TOUCH_PRESSURE_HIGH 1000

/* Number of samples to take for x and y */
#define TOUCH_SAMPLES 16

/* Initialize touchscreen hardware, select the ADC */
/* used_ADC must be one of ADC1 or ADC2. ADC3 currently doesn't work TODO*/
/* 1 = success, 0 = failure */
uint32_t touchscreen_init(ADC_TypeDef *used_ADC);

/* Read raw X position */
uint32_t touchscreen_readx(void);

/* Read raw Y position */
uint32_t touchscreen_ready(void);

/* Read raw pressure */
uint32_t touchscreen_pressure(void);

/* Map the x or y value to screen coordinates (for example) */
/* tlow = touch lowest
 * thigh = touch highest
 * slow = screen lowest
 * shigh = screen highest
 */
int32_t touchscreen_map(uint32_t value, uint32_t tlow, uint32_t thigh, uint32_t slow, uint32_t shigh);

/* 1 if pressed, 0 if not pressed */
uint32_t touchscreen_pressed(uint32_t p);

#endif /* INC_STM32F446_TOUCHSCREEN_H_ */
