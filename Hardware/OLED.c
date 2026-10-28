#include "stm32f10x.h"                  // Device header
#include "My_I2C.h"
#include "OLED.h"
#include "LED.h"

static const uint8_t OLED_InitCmds[] = {
    0x01, 0xAE,         // 关闭显示
    0x02, 0xD5, 0x80,   // 设置时钟分频/振荡器频率
    0x02, 0xA8, 0x3F,   // 多路复用率 (64)
    0x02, 0xD3, 0x00,   // 显示偏移
    0x01, 0x40,         // 显示起始行
    0x02, 0x8D, 0x14,   // 电荷泵
    0x02, 0x20, 0x00,   // 内存寻址模式（水平）
    0x03, 0x21, 0x00, 0x7F,   // ← 新增：设置列地址范围 0~127
    0x03, 0x22, 0x00, 0x07,   // ← 新增：设置页地址范围 0~7
    0x01, 0xA1,         // 段重映射 (左右翻转)
    0x01, 0xC8,         // COM 扫描方向 (上下翻转)
    0x02, 0xDA, 0x12,   // COM 引脚配置
    0x02, 0x81, 0xFF,   // 对比度
    0x02, 0xD9, 0xF1,   // 预充电周期
    0x02, 0xDB, 0x40,   // VCOMH 电压
    0x01, 0xA4,         // 全局显示开启 (RAM 内容)
    0x01, 0xA6,         // 正常显示 (不反转:亮->1,灭->0)
    0x01, 0xAF          // 打开显示
};

/* 简单的 8x16 数字字模 (0-9)，每个字节代表一列，共 16 行 */
static const uint8_t Font8x16[12][16] = {
    {0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00}, // 0
    {0x00,0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00}, // 1
    {0x00,0x70,0x08,0x08,0x08,0x08,0xF0,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00}, // 2
    {0x00,0x30,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x18,0x20,0x21,0x21,0x22,0x1C,0x00}, // 3
    {0x00,0x00,0x80,0x40,0x30,0xF8,0x00,0x00,0x00,0x06,0x05,0x24,0x24,0x3F,0x24,0x24}, // 4
    {0x00,0xF8,0x88,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x20,0x20,0x20,0x11,0x0E,0x00}, // 5
    {0x00,0xE0,0x10,0x88,0x88,0x90,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x20,0x1F,0x00}, // 6
    {0x00,0x18,0x08,0x08,0x88,0x68,0x18,0x00,0x00,0x00,0x00,0x3E,0x01,0x00,0x00,0x00}, // 7
    {0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00}, // 8
    {0x00,0xF0,0x08,0x08,0x08,0x10,0xE0,0x00,0x00,0x01,0x12,0x22,0x22,0x11,0x0F,0x00}, // 9
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x00}, // -
	{0x00,0x00,0x00,0x00,0xE0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x0F,0x01,0x01,0x01}  // +
};


/* 当前初始化进度：指向 OLED_InitCmds 的索引 */
static uint16_t init_index;
static void (*init_done_callback)(void);


/* 发送下一条初始化命令 */
static void OLED_SendNextInitCmd(void);






/* ================= 外部接口API ================= */

/*
 * 初始化 OLED
 * onComplete: 所有初始化命令发送完成后调用
 */
void OLED_Init(void (*onComplete)(void))
{
	init_index = 0;
	init_done_callback = onComplete;
	OLED_SendNextInitCmd();
}

/*
 * 全屏刷新
 * pBuffer: 1024 字节的帧缓冲区
 * onComplete: 刷新完成后回调
 */
void OLED_UpdateScreen(uint8_t *pBuffer, void (*onComplete)(void))
{
	My_I2C_MemWrite(OLED_ADDR, OLED_DATA, pBuffer, OLED_BUFFER_SIZE, onComplete);
}


/*
 * 清空缓冲区 (全部填 0x00)
 */
void OLED_Clear(uint8_t *pBuffer)
{
    for (int i = 0; i < OLED_BUFFER_SIZE; i++) {
        pBuffer[i] = 0x00;
    }
}

/*
 * 在缓冲区中画点
 * color: 1 点亮，0 熄灭
 */
void OLED_SetPixel(uint8_t *pBuffer, uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

    uint16_t index = x + (y / 8) * OLED_WIDTH;
    uint8_t bit = y % 8;

    if (color)
        pBuffer[index] |= (1 << bit);
    else
        pBuffer[index] &= ~(1 << bit);
}


/*============================ 辅助函数 ============================*/
// 字符 -> 字模索引
static uint8_t CharToIndex(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    else if (c == '-') return 10;
    else if (c == '+') return 11;
    else return 0; // 默认显示 '0'
}

/*============================ 核心 API ============================*/
/**
 * 在缓冲区指定位置绘制一个 8x16 字符
 * pBuffer: 指向 1024 字节 OLED 缓冲区的指针
 * x: 列位置 (0 ~ 120，因字宽 8)
 * y: 页位置 (0 ~ 6，因占用两页)
 * ch_idx: 字模数组索引
 */
void OLED_PutChar(uint8_t *pBuffer, uint8_t x, uint8_t y, uint8_t ch_idx) {
    if (x > 120 || y > 6) return;
    const uint8_t *pFont = Font8x16[ch_idx];
    for (uint8_t i = 0; i < 8; i++) {
        // 上半页：当前页
        pBuffer[y * 128 + x + i] = pFont[i];
        // 下半页：下一页
        pBuffer[(y + 1) * 128 + x + i] = pFont[i + 8];
    }
}

/**
 * 显示有符号整数（int32_t 范围）
 * pBuffer: 缓冲区指针
 * x: 起始列
 * y: 起始页（占用两页）
 * num: 要显示的数值
 * show_plus: 非零时正数前显示 '+' 号，零则不显示
 */
void OLED_ShowSignedInt(uint8_t *pBuffer, uint8_t x, uint8_t y, int32_t num, uint8_t show_plus) {
    char buf[12];          // 足够放下 "-2147483648"
    uint8_t len = 0;

    // 处理负号
    if (num < 0) {
        buf[len++] = '-';
        // 安全地取绝对值，避免 -2147483648 溢出
        uint32_t abs_num = (uint32_t)(-(int64_t)num);
        num = (int32_t)abs_num;   // 后续用 abs_num 也可以，但这里直接转回去用 num 继续
    } else if (show_plus) {
        buf[len++] = '+';
    }

    // 将剩余数字转为字符串
    char digits[11];
    uint8_t d_len = 0;
    uint32_t unum = (uint32_t)num;   // 到这里 num 一定是非负
    if (unum == 0) {
        digits[d_len++] = '0';
    } else {
        while (unum > 0) {
            digits[d_len++] = '0' + (unum % 10);
            unum /= 10;
        }
    }

    // 逆序放入 buf
    for (int8_t i = d_len - 1; i >= 0; i--) {
        buf[len++] = digits[i];
    }

    // 逐个字符绘制
    for (uint8_t i = 0; i < len; i++) {
        if (x + i * 8 > 120) break;
        OLED_PutChar(pBuffer, x + i * 8, y, CharToIndex(buf[i]));
    }
}


/**
 * 简化版：显示有符号 int16_t 整数（不带正号）
 * 满足你提出的接口 void OLED_ShowInt(uint8_t *pBuffer, uint8_t x, uint8_t y, int16_t val);
 */
void OLED_ShowInt(uint8_t *pBuffer, uint8_t x, uint8_t y, int16_t val) {
    OLED_ShowSignedInt(pBuffer, x, y, (int32_t)val, 0); // 不显示 '+'，内部安全转换
}



















/* ================= 内部函数 ================= */
static void OLED_SendNextInitCmd(void)
{
	//1.命令发送完毕，进入回调函数
	if(init_index >= sizeof(OLED_InitCmds))
	{
		if(init_done_callback)
		{
			init_done_callback();
		}
		return;
	}
	
	//2.准备从数组中取出发送命令
	uint8_t len = OLED_InitCmds[init_index];   // 本条命令字节数（对应0x01/0x02）
    const uint8_t *pCmd = &OLED_InitCmds[init_index + 1]; // 命令数据首地址
	
	//3.启动非阻塞发送，完成后自动调自己
    My_I2C_MemWrite(OLED_ADDR, OLED_CMD, (uint8_t *)pCmd, len, OLED_SendNextInitCmd);
	
	//4.index跳到下一条命令
	init_index += 1 + len;
}








