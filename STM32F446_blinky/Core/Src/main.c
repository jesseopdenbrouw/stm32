/* p2_1.c Toggle Green LED (LD2) on STM32F446RE Nucleo64 board at 1 Hz
 *
 * This program toggles LD2 for 0.5 second ON and 0.5 second OFF
 * by writing 0 or 1 to bit 5 of the Port A Output Data Register.
 * The green LED (LD2) is connected to PA5.
 * The LED is high active (a '1' turns on the LED).
 * The default system clock is running at 16 MHz.
 *
 * See http://www.microdigitaled.com/ARM/STM_ARM/Code/Ver1/Chapter02/Program2-1.txt
 */

#include "stm32f4xx.h"

void delayMs(int n);

int main(void) {
    RCC->AHB1ENR |=  1;             /* enable GPIOA clock */

    GPIOA->MODER &= ~0x00000C00;    /* clear pin mode (pin PA5) */
    GPIOA->MODER |=  0x00000400;    /* set pin to output mode (pin PA5) */

    while(1) {
        GPIOA->ODR |=  0x00000020;  /* turn on LED (pin PA5) */
        delayMs(500);
        GPIOA->ODR &= ~0x00000020;  /* turn off LED (pin PA5) */
        delayMs(500);
    }
}

/* 16 MHz SYSCLK */
void delayMs(int n) {
    int i;
    for (; n > 0; n--)
        for (i = 0; i < 3195; i++) ;
}
