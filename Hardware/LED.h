#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"                  // Device header

#define LED_CLK RCC_APB2Periph_GPIOA
#define LED_PORT GPIOA
#define LED_PIN GPIO_Pin_4


void LED_Init(void);
void LED_ON(void);
void LED_OFF(void);
void LED_Blink(void);
void LED_Toggle(void);

#endif
