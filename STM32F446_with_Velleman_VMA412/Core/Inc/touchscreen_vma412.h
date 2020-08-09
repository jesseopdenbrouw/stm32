/*
 * Routines for using the Velleman VMA412 touchscreen Graphic LCD
 *
 * Note: pin assignment for STM32F446 Nucleo Board
 *
 * Note: the 8-bit 8080 CPU interface is used.

Software License Agreement (BSD License)

Version: 0.1rc9
Date: 2020/08/08

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

#ifndef INC_TOUCHSCREEN_H_
#define INC_TOUCHSCREEN_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#include "stm32f4xx_hal.h"

/* Driver version */
#define TOUCH_VERSION "STM32 VMA412 TS Driver v 0.1rc6 (Aug 9 2020)"

/* Calibrate the touchscreen, values but be 0 - 1023 */
#define TOUCH_LEFT 120
#define TOUCH_RIGHT 940
#define TOUCH_TOP 900
#define TOUCH_BOTTOM 120
#define TOUCH_PRESSURE_LOW 100
#define TOUCH_PRESSURE_HIGH 1000

/* Number of samples to take for x, y and pressure */
#define TOUCH_SAMPLES 16

/* Initialize touchscreen hardware, select the ADC */
/* used_ADC must be one of ADC1 or ADC2. ADC3 cannot be used, it lacks some common inputs with ADC1 and ADC2 */
/* 1 = success, 0 = failure */
uint32_t touchscreen_init(ADC_TypeDef *used_ADC);

/* Set the ADC speed, use with care */
void touchscreen_setadcspeed(uint32_t speed);

/* Read raw X position */
int32_t touchscreen_readrawx(void);

/* Read raw Y position */
int32_t touchscreen_readrawy(void);

/* Read raw pressure */
int32_t touchscreen_pressure(void);

/* Map the x or y value to screen coordinates (for example) */
/* tlow = touch lowest
 * thigh = touch highest
 * slow = screen lowest
 * shigh = screen highest
 */
int32_t touchscreen_map(int32_t value, int32_t tlow, int32_t thigh, int32_t slow, int32_t shigh);

/* 1 if pressed, 0 if not pressed */
uint32_t touchscreen_ispressed(uint32_t p);

#ifdef __cplusplus
}
#endif

#endif /* INC_TOUCHSCREEN_H_ */
