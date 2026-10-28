#include "stm32f10x.h"                  // Device header
#include "My_I2C.h"
#include "LED.h"

/* ========== 操作模式 ========== */
typedef enum {
    OP_WRITE = 0,
    OP_READ,
    OP_MEM_WRITE,
    OP_MEM_READ
}I2C_OpMode;



/* ========== 状态机内部阶段定义(章节) ========== */
typedef enum
{
	STATE_IDLE = 0,
	STATE_START,
	STATE_SEND_ADDR,		 //	发送写地址 (设备地址+W)
	STATE_WAIT_ACK_ADDR,	
	STATE_SEND_REG,          // 发送寄存器地址 (Mem 操作专用)
    STATE_WAIT_ACK_REG,
	STATE_SEND_DATA,		 //	发送数据
	STATE_WAIT_ACK_DATA,
	STATE_RESTART,           // 重复起始条件
	STATE_SEND_ADDR_RD,      // 发送读地址 (设备地址+R)
	STATE_WAIT_ACK_ADDR_RD,
	STATE_RECV_DATA,         // 接收数据
    STATE_SEND_ACK,          // 发送 ACK / NACK
	STATE_STOP,
	STATE_FINISH
}MainState;

/* 子状态：每个阶段内部更细致的动作(页码) */
typedef enum
{
	SUB_IDLE = 0,
	
	/* 起始条件子状态 */
	SUB_START_SDA_H,
	SUB_START_SDA_L,
	SUB_START_SCL_L,
	
	/* 重复起始条件 */
    SUB_RESTART_SDA_H,
    SUB_RESTART_SCL_H,
    SUB_RESTART_SDA_L,
    SUB_RESTART_SCL_L,
    SUB_RESTART_DONE,
	
	/* 发送一个字节的子状态（地址/寄存器/数据通用） */
	SUB_SEND_SDA,
	SUB_SEND_SCL_H,
	SUB_SEND_SCL_L,
	SUB_SEND_NEXT_BIT,
	
	/* 等待ACK的子状态 */
	SUB_ACK_RELEASE_SDA,
    SUB_ACK_SCL_H,
    SUB_ACK_SCL_L,
    SUB_ACK_DONE,
	
	/* 接收一个字节 */
	SUB_RECV_RELEASE_SDA,
	SUB_RECV_SCL_H,
	SUB_RECV_SCL_L,
	
	/* 发送 ACK / NACK */
	SUB_SENDACK_SDA,
	SUB_SENDACK_SCL_H,
	SUB_SENDACK_SCL_L,
	SUB_SENDACK_RELEASE_SDA,
	
	/* 停止条件子状态 */
    SUB_STOP_SDA_L,
    SUB_STOP_SCL_H,
    SUB_STOP_SDA_H
}SubState;

/* ========== 全局状态上下文 ========== */
static struct {
    MainState state;
    SubState  subState;
	
    uint16_t  delay;          	// 还需要等待的 tick 数
    uint8_t   devAddr;        	// 7位设备地址
	uint8_t   regAddr;        	// 寄存器地址 (Mem 操作)
    uint8_t   *pData;			// 数据缓冲区
    uint16_t   length;			// 数据长度
    uint16_t   index;          	// 当前数据字节索引
    uint8_t   bitIndex;       	// 当前字节内的位索引 (0~7)
    uint8_t   sendByte;       	// 正在操作（发送/接收）的字节
    uint8_t   ack;            	// 收到的ACK标志 (0=ACK, 1=NACK)
	uint8_t   isLastByte;       // 接收时标记是否最后一个字节 (用于发 NACK)
	I2C_OpMode opMode;
    void (*onComplete)(void); 	// 完成回调
}g_i2c;



/* ========== 内部辅助函数声明 ========== */
static void SetDelay(uint16_t ticks, SubState nextSub);
static void Process_Start(void);
static void Process_SendByte(void);
static void Process_WaitAck(void);
static void Process_Stop(void);
static void Process_Restart(void);
static void Process_RecvData(void);
static void Process_SendAck(void);
static void Process_Finish(void);


/* 辅助：设置一次延时并切换到下一个子状态 */
static inline void SetDelay(uint16_t ticks, SubState nextSub)
{
    g_i2c.delay    = ticks;
    g_i2c.subState = nextSub;
}


/* ========== 查询忙状态 ========== */
uint8_t I2C_Busy(void)
{
    return (g_i2c.state != STATE_IDLE) ? 1 : 0;
}



/* ========== GPIO 初始化 ========== */
void My_I2C_Init(void)
{
	RCC_APB2PeriphClockCmd(MY_I2C_CLK,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Pin = MY_I2C_SCL_PIN|MY_I2C_SDA_PIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MY_I2C_PORT,&GPIO_InitStruct);
	
	//释放总线
	SCL_H();
	SDA_H();
	
	//初始化状态机变量
	g_i2c.state = STATE_IDLE;
	g_i2c.subState = SUB_IDLE;
	g_i2c.delay = 0;
}




/* ===============================API==================================== */

void My_I2C_MasterTransmit(uint8_t devAddr7, uint8_t *pData, uint16_t len,
                           void (*callback)(void))
{
    if (g_i2c.state != STATE_IDLE) return;		//忙保护
    g_i2c.devAddr    = devAddr7;
    g_i2c.pData      = pData;
    g_i2c.length     = len;
    g_i2c.onComplete = callback;
    g_i2c.opMode     = OP_WRITE;
    g_i2c.index      = 0;
    g_i2c.bitIndex   = 0;
    g_i2c.state      = STATE_START;
    g_i2c.subState   = SUB_START_SDA_H;
    g_i2c.delay      = 0;
}

void My_I2C_MasterReceive(uint8_t devAddr7, uint8_t *pData, uint16_t len,
                        void (*callback)(void))
{
	if (g_i2c.state != STATE_IDLE) return;		//忙保护
    g_i2c.devAddr    = devAddr7;
    g_i2c.pData      = pData;
    g_i2c.length     = len;
    g_i2c.onComplete = callback;
    g_i2c.opMode     = OP_READ;
    g_i2c.index      = 0;
    g_i2c.bitIndex   = 0;
    g_i2c.state      = STATE_START;
    g_i2c.subState   = SUB_START_SDA_H;
    g_i2c.delay      = 0;		
}	

void My_I2C_MemWrite(uint8_t devAddr7, uint8_t RegisterAddress, uint8_t *pData, uint16_t len, 
                        void (*callback)(void))
{
	if (g_i2c.state != STATE_IDLE) return;		//忙保护
    g_i2c.devAddr    = devAddr7;
	g_i2c.regAddr    = RegisterAddress;
    g_i2c.pData      = pData;
    g_i2c.length     = len;
    g_i2c.onComplete = callback;
    g_i2c.opMode     = OP_MEM_WRITE;
    g_i2c.index      = 0;
    g_i2c.bitIndex   = 0;
    g_i2c.state      = STATE_START;
    g_i2c.subState   = SUB_START_SDA_H;
    g_i2c.delay      = 0;		
}
						
void My_I2C_MemRead(uint8_t devAddr7, uint8_t RegisterAddress, uint8_t *pData, uint16_t len, 
                        void (*callback)(void))
{
	if (g_i2c.state != STATE_IDLE) return;		//忙保护
    g_i2c.devAddr    = devAddr7;
	g_i2c.regAddr    = RegisterAddress;
    g_i2c.pData      = pData;
    g_i2c.length     = len;
    g_i2c.onComplete = callback;
    g_i2c.opMode     = OP_MEM_READ;
    g_i2c.index      = 0;
    g_i2c.bitIndex   = 0;
    g_i2c.state      = STATE_START;
    g_i2c.subState   = SUB_START_SDA_H;
    g_i2c.delay      = 0;		
}
















/* ========== 各个阶段的处理函数 ========== */
/* 起始条件 */
static void Process_Start(void)
{
	switch (g_i2c.subState)
	{
		case SUB_START_SDA_H:
			SCL_H();
			SDA_H();
			SetDelay(I2C_US_TO_TICKS(3),SUB_START_SDA_L);
			break;
		case SUB_START_SDA_L:
			SDA_L();
			SetDelay(I2C_US_TO_TICKS(3),SUB_START_SCL_L);
			break;
		case SUB_START_SCL_L:
			SCL_L();
			SetDelay(I2C_US_TO_TICKS(3),SUB_IDLE);
			//根据opMode决定下一步
			if(g_i2c.opMode == OP_READ)
			{
				// 纯读：直接发送读地址
                g_i2c.state    = STATE_SEND_ADDR_RD;
                g_i2c.sendByte = (g_i2c.devAddr << 1) | 0x01;
			}
			else
			{
				//其他模式，先发送写地址
				g_i2c.state    = STATE_SEND_ADDR;
				g_i2c.sendByte = (g_i2c.devAddr << 1) & 0xFE;
			}
			g_i2c.bitIndex = 0;
            g_i2c.subState = SUB_SEND_SDA;
            g_i2c.delay    = 1;	
            break;
        default: break;
	}
}

/* 发送一个字节（地址/数据通用） */
static void Process_SendByte(void)
{
	switch (g_i2c.subState)
	{
		case SUB_SEND_SDA:
			if(g_i2c.sendByte & 0x80)	SDA_H();
			else						SDA_L();
			SetDelay(I2C_US_TO_TICKS(2),SUB_SEND_SCL_H);
			break;
		case SUB_SEND_SCL_H:
			SCL_H();
			SetDelay(I2C_US_TO_TICKS(3),SUB_SEND_SCL_L);
			break;
		case SUB_SEND_SCL_L:
			SCL_L();
			SetDelay(I2C_US_TO_TICKS(3),SUB_SEND_NEXT_BIT);
			break;
		case SUB_SEND_NEXT_BIT:
			g_i2c.sendByte <<= 1;
			g_i2c.bitIndex ++;
			if(g_i2c.bitIndex >= 8)
			{
				// 发送完成，根据当前主状态进入对应的等待 ACK 阶段
                switch(g_i2c.state)
				{
					case STATE_SEND_ADDR:    g_i2c.state = STATE_WAIT_ACK_ADDR;    break;
                    case STATE_SEND_REG:     g_i2c.state = STATE_WAIT_ACK_REG;     break;
                    case STATE_SEND_DATA:    g_i2c.state = STATE_WAIT_ACK_DATA;    break;
                    case STATE_SEND_ADDR_RD: g_i2c.state = STATE_WAIT_ACK_ADDR_RD; break;
                    default: break;
				}
				g_i2c.subState = SUB_ACK_RELEASE_SDA;
                g_i2c.delay    = 1;	
			}
			else
			{
                SetDelay(I2C_US_TO_TICKS(3),SUB_SEND_SDA);
			}
			break;
		default: break;
	}
}

/* 等待从机ACK */
static void Process_WaitAck(void)
{
	switch (g_i2c.subState)
	{
		case SUB_ACK_RELEASE_SDA:
			SDA_H();
			SetDelay(I2C_US_TO_TICKS(3),SUB_ACK_SCL_H);
			break;
		case SUB_ACK_SCL_H:
			SCL_H();
			SetDelay(I2C_US_TO_TICKS(3),SUB_ACK_SCL_L);
			break;
		case SUB_ACK_SCL_L:
			g_i2c.ack = SDA_READ() ? 1 : 0;
			SCL_L();
			SetDelay(I2C_US_TO_TICKS(3),SUB_ACK_DONE);	
			break;
		case SUB_ACK_DONE:
			// ACK 完成，根据当前STATE和OPMODE跳转
            switch( g_i2c.state )
			{
				case STATE_WAIT_ACK_ADDR:
					if(g_i2c.opMode == OP_WRITE)
					{
						if (g_i2c.length > 0) 
						{
                            g_i2c.state    = STATE_SEND_DATA;
							g_i2c.subState = SUB_SEND_SDA;
                            g_i2c.sendByte = g_i2c.pData[0];
                            g_i2c.index    = 0;                    
                            g_i2c.bitIndex = 0;
                            g_i2c.delay    = 1;
                        }
						else
						{
							g_i2c.state    = STATE_STOP;
                            g_i2c.subState = SUB_STOP_SDA_L;
                            g_i2c.delay    = 1;
						}
					}
					else if((g_i2c.opMode == OP_MEM_WRITE || g_i2c.opMode == OP_MEM_READ))
					{
						// Mem 操作：发送寄存器地址
						g_i2c.state    = STATE_SEND_REG;
                        g_i2c.subState = SUB_SEND_SDA;
						g_i2c.sendByte = g_i2c.regAddr;
                        g_i2c.bitIndex = 0;
                        g_i2c.delay    = 1;
					}
					break;
				case STATE_WAIT_ACK_REG:
					if(g_i2c.opMode == OP_MEM_WRITE)
					{
						// 发完寄存器地址，开始发送数据
						if(g_i2c.length > 0)
						{
							g_i2c.state = STATE_SEND_DATA;
							g_i2c.subState = SUB_SEND_SDA;
							g_i2c.sendByte = g_i2c.pData[0];
							g_i2c.index    = 0;     
                            g_i2c.bitIndex = 0;
                            g_i2c.delay    = 1;
						}
						else
						{
							g_i2c.state = STATE_STOP;
							g_i2c.subState = SUB_STOP_SDA_L;
							g_i2c.delay    = 1;
						}
					}
					else if (g_i2c.opMode == OP_MEM_READ)
					{
						// 发完寄存器地址，进入重复起始条件
						g_i2c.state    = STATE_RESTART;
                        g_i2c.subState = SUB_RESTART_SDA_H;
                        g_i2c.delay    = 1;
					}
					break;
				case STATE_WAIT_ACK_DATA:
					g_i2c.index++;
                    if (g_i2c.index < g_i2c.length)
					{
                        g_i2c.sendByte = g_i2c.pData[g_i2c.index];
                        g_i2c.state    = STATE_SEND_DATA;
                        g_i2c.subState = SUB_SEND_SDA;
                        g_i2c.bitIndex = 0;
                        g_i2c.delay    = 1;
                    } 
					else 
					{
                        g_i2c.state    = STATE_STOP;
                        g_i2c.subState = SUB_STOP_SDA_L;
                        g_i2c.delay    = 1;
                    }
                    break;
				case STATE_WAIT_ACK_ADDR_RD:
					// 读地址ACK后，进入接收数据
					if (g_i2c.length > 0)
					{
                        g_i2c.state    = STATE_RECV_DATA;
						g_i2c.subState = SUB_RECV_RELEASE_SDA;
                        g_i2c.index    = 0;
                        g_i2c.bitIndex = 0;  
                        g_i2c.delay    = 1;
                    }
					else
					{
						g_i2c.state    = STATE_STOP;
                        g_i2c.subState = SUB_STOP_SDA_L;
                        g_i2c.delay    = 1;
					}
					break;
				default: break;
			}
			break;
		default: break;
	}
}


/* ========== 重复起始 ========== */
static void Process_Restart(void)
{
	switch(g_i2c.subState)
	{
		case SUB_RESTART_SDA_H:
			SDA_H();
			SetDelay(I2C_US_TO_TICKS(3), SUB_RESTART_SCL_H);
			break;
		case SUB_RESTART_SCL_H:
			SCL_H();
			SetDelay(I2C_US_TO_TICKS(3), SUB_RESTART_SDA_L);
			break;
		case SUB_RESTART_SDA_L:
			SDA_L();
			SetDelay(I2C_US_TO_TICKS(3), SUB_RESTART_SCL_L);
			break;
		case SUB_RESTART_SCL_L:
			SCL_L();
			SetDelay(I2C_US_TO_TICKS(3), SUB_RESTART_DONE);
			break;
		case SUB_RESTART_DONE:
			// 重复起始完成，发送读地址
			g_i2c.state    = STATE_SEND_ADDR_RD;
			g_i2c.subState = SUB_SEND_SDA;
            g_i2c.sendByte = (g_i2c.devAddr << 1) | 0x01;
            g_i2c.bitIndex = 0;
            g_i2c.delay    = 1;
            break;
		default: break;	
	}
}


/* ========== 接收数据 ========== */
static void Process_RecvData(void)
{
	switch(g_i2c.subState)
	{
		case SUB_RECV_RELEASE_SDA:
			SDA_H();
			SetDelay(I2C_US_TO_TICKS(2), SUB_RECV_SCL_H);
			break;
		case SUB_RECV_SCL_H:
			SCL_H();
			SetDelay(I2C_US_TO_TICKS(3), SUB_RECV_SCL_L);
			break;
		case SUB_RECV_SCL_L:
			g_i2c.sendByte <<= 1;
			if(SDA_READ())	g_i2c.sendByte |= 0x01;
			SCL_L();
			SetDelay(I2C_US_TO_TICKS(3), SUB_RECV_SCL_H);  // 等下一位
			g_i2c.bitIndex ++;
			if(g_i2c.bitIndex >= 8)
			{
				//一个字节接收完成
				g_i2c.pData[g_i2c.index] = g_i2c.sendByte;	//接收字节放进数据缓冲区
				g_i2c.index ++;
				// 判断是否最后一个字节
				if(g_i2c.index >= g_i2c.length)
				{
					g_i2c.isLastByte = 1;   // 最后一个字节，发 NACK
				}
				else
				{
					g_i2c.isLastByte = 0;   // 中间字节，发 ACK
				}
				// 进入发送 ACK/NACK 阶段
				g_i2c.state    = STATE_SEND_ACK;
				g_i2c.subState = SUB_SENDACK_SDA;
				g_i2c.delay    = 1;
			}
			break;
		default: break;
	}
}



/* ========== 发送 ACK/NACK ========== */
static void Process_SendAck(void)
{
	switch(g_i2c.subState)
	{
		case SUB_SENDACK_SDA:
			if(g_i2c.isLastByte)	SDA_H();	//NACK
			else					SDA_L();	//ACK
			SetDelay(I2C_US_TO_TICKS(2), SUB_SENDACK_SCL_H);
            break;
		case SUB_SENDACK_SCL_H:
			SCL_H();
			SetDelay(I2C_US_TO_TICKS(3), SUB_SENDACK_SCL_L);
			break;
		case SUB_SENDACK_SCL_L:
			SCL_L();
			SetDelay(I2C_US_TO_TICKS(2), SUB_SENDACK_RELEASE_SDA);
			break;
		case SUB_SENDACK_RELEASE_SDA:
			SDA_H();
			// 判断后续动作
			if(g_i2c.isLastByte)
			{
				// 发完 NACK 后进入停止
                g_i2c.state    = STATE_STOP;
                g_i2c.subState = SUB_STOP_SDA_L;
			}
			else 
			{
				// 发完 ACK 后继续接收下一个字节
                g_i2c.state    = STATE_RECV_DATA;
                g_i2c.subState = SUB_RECV_RELEASE_SDA;
                g_i2c.bitIndex = 0;
                g_i2c.sendByte = 0;
			}
			g_i2c.delay = 1;
            break;
		default: break;
	}
}




/* 停止条件 */
static void Process_Stop(void)
{
	switch (g_i2c.subState)
	{
		case SUB_STOP_SDA_L:
			SDA_L();
			SetDelay(I2C_US_TO_TICKS(3),SUB_STOP_SCL_H);
			break;
		case SUB_STOP_SCL_H:
			SCL_H();
			SetDelay(I2C_US_TO_TICKS(3),SUB_STOP_SDA_H);
			break;
		case SUB_STOP_SDA_H:
			SDA_H();
			SetDelay(I2C_US_TO_TICKS(3),SUB_IDLE);
			g_i2c.state = STATE_FINISH;
			g_i2c.delay = 1;
			break;
		default: break;
	}
}

/* 完成处理 */
static void Process_Finish(void)
{
	g_i2c.state = STATE_IDLE;   // 状态机回到空闲
		
    if (g_i2c.onComplete) {
        g_i2c.onComplete();
    }
    
}


/* ========== 核心：每个 tick 调用一次 ========== */
void My_I2C_Tick_Handler(void)
{
	// 1. 如果当前处于延时等待，递减计数器后直接退出( Tick_Delay完成的关键 )
    if (g_i2c.delay > 0) {
        g_i2c.delay--;
        return;
    }
	
	// 2. 根据主状态分发处理
	switch (g_i2c.state)
	{
		case STATE_IDLE:         		break;
		case STATE_START:				Process_Start();	break;
		case STATE_SEND_ADDR:			/* fall through */
		case STATE_SEND_REG:			/* fall through */
		case STATE_SEND_DATA:			/* fall through */
		case STATE_SEND_ADDR_RD:		Process_SendByte();	break;
		case STATE_WAIT_ACK_ADDR:		/* fall through */
		case STATE_WAIT_ACK_REG:		/* fall through */
		case STATE_WAIT_ACK_DATA:		/* fall through */
		case STATE_WAIT_ACK_ADDR_RD:	Process_WaitAck();	break;
		case STATE_RESTART:				Process_Restart();	break;
		case STATE_RECV_DATA:			Process_RecvData();	break;
		case STATE_SEND_ACK:			Process_SendAck();	break;
		case STATE_STOP:				Process_Stop();		break;
		case STATE_FINISH:				Process_Finish();	break;
		default:	break;
	}
}

