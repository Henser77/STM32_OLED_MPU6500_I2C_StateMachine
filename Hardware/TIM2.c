#include "stm32f10x.h"                  // Device header


void TIM2_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 72-1;
	TIM2->ARR = 3-1;
	TIM2->EGR = TIM_EGR_UG;
	TIM2->SR = 0x0000;
	TIM2->DIER = TIM_DIER_UIE;
	
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn,1);
	
	TIM2->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
}

