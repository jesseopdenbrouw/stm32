/* p2_3.c Turn on or off LED by a switch
 *
 * This program turns on the green LED (LD2) by pressing the user
 * button B1 of the Nucleo board.
 * The user button is connected to PC13. It has a pull-up resitor
 * so PC13 stays high when the button is not pressed.
 * When the button is pressed, PC13 becomes low.
 * The green LED (LD2) is connected to PA5.
 * A high on PA5 turns on the LED.
 *
 * See http://www.microdigitaled.com/ARM/STM_ARM/Code/Ver1/Chapter02/Program2-3.txt
 */

#include "stm32f4xx.h"

int main(void) {
    RCC->AHB1ENR |=  4;                 /* enable GPIOC clock */
    RCC->AHB1ENR |=  1;                /* enable GPIOA clock */

    GPIOA->MODER &= ~0x00000C00;        /* clear pin mode */
    GPIOA->MODER |=  0x00000400;        /* set pin to output mode */

    GPIOC->MODER &= ~0x0C000000;        /* clear pin mode to input mode */

    while(1) {
        if (GPIOC->IDR & 0x2000)        /* if PC13 is high */
            GPIOA->BSRR = 0x00200000;   /* turn off green LED */
        else
            GPIOA->BSRR = 0x00000020;   /* turn on green LED */
    }
}

