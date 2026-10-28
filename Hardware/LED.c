#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "LED.h"

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(LED_CLK,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = LED_PIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LED_PORT,&GPIO_InitStruct);
	LED_OFF();
}

void LED_ON(void)
{
	GPIO_ResetBits(LED_PORT,LED_PIN);
}

void LED_OFF(void)
{
	GPIO_SetBits(LED_PORT,LED_PIN);
}

void LED_Blink(void)
{
	while(1)
	{
		LED_ON();
		Delay_ms(50);
		LED_OFF();
		Delay_ms(50);
	}
	
}

void LED_Toggle(void)
{
	if(GPIO_ReadInputDataBit(LED_PORT,LED_PIN)==0)
	{
		LED_OFF();
	}
	else
	{
		LED_ON();
	}
}
