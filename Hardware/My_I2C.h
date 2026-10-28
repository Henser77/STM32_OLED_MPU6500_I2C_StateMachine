#ifndef __MY_I2C_H
#define __MY_I2C_H

#include "stm32f10x.h"                  // Device header


/* ========== 硬件引脚配置 ========== */
#define MY_I2C_CLK 		RCC_APB2Periph_GPIOB
#define MY_I2C_PORT		GPIOB
#define MY_I2C_SCL_PIN 	GPIO_Pin_6
#define MY_I2C_SDA_PIN 	GPIO_Pin_7


/* ========== 定时器中断周期 (us)，需与实际定时器周期一致 ========== */
#define I2C_TICK_US     3  // 如果实际定时器是3us，就保持3


/* 将微秒转换为 tick 数（向上取整） */
#define I2C_US_TO_TICKS(us)  (((us) + (I2C_TICK_US) - 1) / (I2C_TICK_US))


/* ========== 引脚宏 ========== */
#define SCL_H()	GPIO_SetBits(MY_I2C_PORT,MY_I2C_SCL_PIN)
#define SCL_L()	GPIO_ResetBits(MY_I2C_PORT,MY_I2C_SCL_PIN)
#define SDA_H()	GPIO_SetBits(MY_I2C_PORT,MY_I2C_SDA_PIN)
#define SDA_L()	GPIO_ResetBits(MY_I2C_PORT,MY_I2C_SDA_PIN)
#define SDA_READ()	GPIO_ReadInputDataBit(MY_I2C_PORT,MY_I2C_SDA_PIN)


/* ========== API 声明 ========== */
void My_I2C_Init(void);
void My_I2C_Tick_Handler(void);
uint8_t I2C_Busy(void);


/*----------纯写传输--------------*/
void My_I2C_MasterTransmit(uint8_t devAddr7, uint8_t *pData, uint16_t len,
                        void (*callback)(void));

/*----------纯读传输(不发送寄存器地址)--------------*/
void My_I2C_MasterReceive(uint8_t devAddr7,uint8_t *pData,uint16_t len,
						void (*callback)(void));

/*----------内层写：指定寄存器写数据--------------*/
void My_I2C_MemWrite(uint8_t devAddr7, uint8_t RegisterAddress, uint8_t *pData,uint16_t len,
                        void (*callback)(void));

/*----------内层读：指定寄存器读数据--------------*/
void My_I2C_MemRead(uint8_t devAddr7, uint8_t RegisterAddress, uint8_t *pData,uint16_t len,
                        void (*callback)(void));

#endif
