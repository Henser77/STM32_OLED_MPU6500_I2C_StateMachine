#ifndef __OLED_H
#define __OLED_H

#include "stm32f10x.h"                  // Device header

/* OLED I2C 地址 ( 0x3C/0x3D,一般0x3C ) */
#define	OLED_ADDR	0x3C

/* 控制字节 */
#define OLED_CMD    0x00   // 下个字节是命令
#define OLED_DATA   0x40   // 下个字节是数据

/* 屏幕尺寸 */
#define OLED_WIDTH  128
#define OLED_HEIGHT 64

/* 全屏缓冲大小 (128*64/8 = 1024) */
#define OLED_BUFFER_SIZE  1024

/* --------------- API ---------------------- */
void OLED_Init(void (*onComplete)(void));
void OLED_UpdateScreen(uint8_t *pBuffer, void (*onComplete)(void));
void OLED_Clear(uint8_t *pBuffer);
void OLED_SetPixel(uint8_t *pBuffer, uint8_t x, uint8_t y, uint8_t color);
void OLED_PutChar(uint8_t *pBuffer, uint8_t x, uint8_t y, uint8_t ch_idx);
void OLED_ShowSignedInt(uint8_t *pBuffer, uint8_t x, uint8_t y, int32_t num, uint8_t show_plus);
void OLED_ShowInt(uint8_t *pBuffer, uint8_t x, uint8_t y, int16_t val);


#endif
