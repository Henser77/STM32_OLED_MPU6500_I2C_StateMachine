#include "MPU6500.h"
#include "My_I2C.h"

static const uint8_t MPU6500_InitCmds[] = {
	0x02,	PWR_MGMT_1, 	0x00,
	0x02,	ACCEL_CONFIG, 	0x00,
	0x02,	GYRO_CONFIG,	0x00,
};


/* 当前初始化进度：指向 MPU6500_InitCmds 的索引 */
static uint16_t init_index;
static void (*init_done_callback)();


/* 发送下一条初始化命令 */
static void MPU6500_SendNextInitCmd(void);





/* ================= 外部接口 ================= */

/*
 * 初始化 MPU6500
 * onComplete: 所有初始化命令发送完成后调用
 */
void MPU6500_Init(void (*callback)(void))
{
	init_index = 0;
	init_done_callback = callback;
	MPU6500_SendNextInitCmd();
}


void MPU6500_ReadAll(uint8_t *buf, void (*callback)(void))
{
	My_I2C_MemRead(MPU6500_ADDR, ACCEL_XOUT_H, buf, 14, callback);
}

void MPU6500_ParseAccel(uint8_t *buf, int16_t *ax, int16_t *ay, int16_t *az)
{
    *ax = (buf[0] << 8) | buf[1];
    *ay = (buf[2] << 8) | buf[3];
    *az = (buf[4] << 8) | buf[5];
}

void MPU6500_ParseGyro(uint8_t *buf, int16_t *gx, int16_t *gy, int16_t *gz)
{
    *gx = (buf[8]  << 8) | buf[9];
    *gy = (buf[10] << 8) | buf[11];
    *gz = (buf[12] << 8) | buf[13];
}





/* ================= 内部函数 ================= */
static void MPU6500_SendNextInitCmd(void)
{
	//1.命令发送完毕，进入回调函数
	if(init_index >= sizeof(MPU6500_InitCmds))
	{
		if(init_done_callback)
		{
			init_done_callback();
		}
		return;
	}
	
	//2.准备从数组中取出发送命令
	uint8_t len = MPU6500_InitCmds[init_index];   // 本条命令字节数（对应0x02）
	//const修饰：该指针指向的不可改变（uint8_t * const pCmd：该指针不可指向其他）
    const uint8_t *pCmd = &MPU6500_InitCmds[init_index + 1]; // 写入寄存器的数据首地址
	
	//3.启动非阻塞发送，完成后自动调自己
    My_I2C_MemWrite(MPU6500_ADDR, pCmd[0], (uint8_t *)&pCmd[1], len-1, MPU6500_SendNextInitCmd);
	
	//4.index跳到下一条命令
	init_index += 1 + len;
}
