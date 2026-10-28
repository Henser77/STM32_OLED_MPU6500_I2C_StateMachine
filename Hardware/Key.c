#include "stm32f10x.h"                  // Device header
#include "Key.h"


#define	KEY_CLK 	RCC_APB2Periph_GPIOB//按键时钟
#define KEY_PORT 	GPIOB				//按键外设
#define	KEY_PIN		GPIO_Pin_0			//按键端口

#define KEY_DEBOUNCE_TIME	20		//消抖时间20*1ms = 20ms
#define KEY_LONG_TIME		1000	//长按1000*1ms = 1s

#define KEY_READ()		(!GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN))	//1为按下，0为释放


//按键状态枚举
typedef enum{
	KEY_STATE_IDLE = 0,			//闲置状态
	KEY_STATE_DEBOUNCE_DOWN,	//按下消抖
	KEY_STATE_PRESS,			//按下
	KEY_STATE_DEBOUNCE_UP		//释放消抖
}Key_State_t;



//按键初始化
void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(KEY_CLK,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = KEY_PIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(KEY_PORT,&GPIO_InitStruct);
}


//按键扫描
uint8_t Key_Scan(void)
{
	static uint32_t cnt=0;						//唯一按键状态计数变量
	static Key_State_t state = KEY_STATE_IDLE;	
	static uint8_t LONG_READY = 0;
	uint8_t event = KEY_EVENT_NONE;
	uint8_t cur_level = KEY_READ();				
	
	
	switch(state)
	{
		case KEY_STATE_IDLE:
			if(cur_level)
			{
				state = KEY_STATE_DEBOUNCE_DOWN;//检测到按下立刻进入按下消抖
				cnt = 0;
			}
		break;
			
		case KEY_STATE_DEBOUNCE_DOWN:
			cnt ++;
			if(cnt>=KEY_DEBOUNCE_TIME)
			{
				cnt = 0;
				if(cur_level)						//确定还是按下而不是抖动
				{
					state = KEY_STATE_PRESS;		//按下消抖完成进入按下状态
				}else state = KEY_STATE_IDLE;		//如果是抖动则回到闲置状态
			}
		break;
			
		case KEY_STATE_PRESS:
			cnt ++;
			if(!cur_level)
			{
				cnt = 0;
				state = KEY_STATE_DEBOUNCE_UP;	//触发长按之前检测到释放进入释放消抖
			}
			else if(cnt == KEY_LONG_TIME)
			{
				event = KEY_EVENT_LONG;			//触发长按状态
				LONG_READY = 1;
				cnt = KEY_LONG_TIME + 1;		//防止重复触发长按
			}
		break;
			
		case KEY_STATE_DEBOUNCE_UP:
			cnt ++;
			if(LONG_READY)
			{
				if(cnt>=KEY_DEBOUNCE_TIME)
				{
					cnt = 0;
					LONG_READY = 0;
					state = KEY_STATE_IDLE;
				}
			}
			else
			{
				if(cur_level)
				{
					state = KEY_STATE_DEBOUNCE_DOWN;//释放消抖期间检测到按下立刻进入按下消抖
					cnt = 0;
				}
				else if(cnt>=KEY_DEBOUNCE_TIME)
				{
					cnt = 0;
					state = KEY_STATE_IDLE;			//释放消抖结束回到闲置状态
					event = KEY_EVENT_SHORT;		//释放消抖结束置为短按
				}
			}
			
		break;
			
		default:
			cnt = 0;
			state = KEY_STATE_IDLE;
			event = KEY_EVENT_NONE;	
		break;
	}
		
	return event;	
				
}

