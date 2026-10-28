#include "stm32f10x.h"                  // Device header
#include "My_I2C.h"
#include "OLED.h"
#include "MPU6500.h"
#include "LED.h"
#include "Key.h"
#include "TIM2.h"
#include <stddef.h>
#include "math.h"


#define M_PI 3.14159265358979323846f



/*--------------------全局变量定义和函数声明------------------------*/
volatile uint32_t Systick_Tick = 0;		//SysTick中断基准计数变量
static uint32_t Key_Lasttime_Tick = 0;		//按键上次任务执行记录
static uint32_t OLED_Lasttime_Tick = 0;		//OLED刷新请求上次置位执行记录

volatile uint8_t OLED_Update_Request = 0;
volatile uint8_t MPU6500_Read_Request = 1;


void Key_Task(void);
void TryNextTransfer(void);	//调度器原型

float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
uint32_t last_mpu_tick = 0;   // 用于计算 dt



/* 三个显示页面的数据缓存（放大后的整数） */
int16_t accel_display[3];    // 加速度 (mg，乘以 1000)
int16_t gyro_display[3];     // 角速度 (0.1°/s，乘以 10)
int16_t attitude_display[3]; // 姿态角 (°，直接取整)

uint8_t display_page = 0;    // 当前页面：0=加速度, 1=角速度, 2=姿态角
uint8_t clear_request = 0;   // 长按清零标志





/*--------------------------MPU6500应用----------------------------------*/
uint8_t mpu_buf[14];
int16_t accel_buf[3];
int16_t gyro_buf[3];


void MPU_Ready(void)
{
    TryNextTransfer();
}

void MPU_DataReady(void)
{
	//取得原始数据LSB
	MPU6500_ParseAccel(mpu_buf,&accel_buf[0],&accel_buf[1],&accel_buf[2]);
	MPU6500_ParseGyro(mpu_buf,&gyro_buf[0],&gyro_buf[1],&gyro_buf[2]);
	
	/* ------------处理LSB--------------*/
	// 转换为物理量
    float ax = accel_buf[0] / 16384.0f;
    float ay = accel_buf[1] / 16384.0f;
    float az = accel_buf[2] / 16384.0f;

    float gx = gyro_buf[0] / 131.0f;   // 注意单位 °/s
    float gy = gyro_buf[1] / 131.0f;
	float gz = gyro_buf[2] / 131.0f;
	
	// 计算 dt
    uint32_t now = Systick_Tick;
    float dt = (now - last_mpu_tick) / 1000.0f;   // ms → s
    last_mpu_tick = now;
	
	// 从加速度计计算倾角
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    float roll_acc  = atan2f(ay, az) * 180.0f / M_PI;

    // 一阶互补滤波
    float alpha = 0.98f;
    pitch = alpha * (pitch + gx * dt) + (1.0f - alpha) * pitch_acc;
    roll  = alpha * (roll  + gy * dt) + (1.0f - alpha) * roll_acc;
	yaw   += gz * dt;   // 纯积分，会漂移


	// 长按清零偏航角
    if (clear_request) {
        yaw = 0.0f;
        clear_request = 0;
    }

	// 填充显示数组
    accel_display[0] = (int16_t)(ax * 1000);   // mg
    accel_display[1] = (int16_t)(ay * 1000);
    accel_display[2] = (int16_t)(az * 1000);

    gyro_display[0] = (int16_t)(gx * 10);      // 0.1°/s
    gyro_display[1] = (int16_t)(gy * 10);
    gyro_display[2] = (int16_t)(gz * 10);

    attitude_display[0] = (int16_t)pitch;
    attitude_display[1] = (int16_t)roll;
    attitude_display[2] = (int16_t)yaw;

	
	
	TryNextTransfer();	//回到调度器
}

/*-------------------------OLED应用---------------------------*/
static uint8_t oled_buffer[OLED_BUFFER_SIZE];

void OLED_InitDone(void)
{
	OLED_Update_Request = 1;       // 请求刷新
	
    MPU6500_Init(MPU_Ready);       // 接着初始化 MPU6500
}


void OLED_DrawPage(void)
{
    OLED_Clear(oled_buffer);

    switch(display_page) {
        case 0: // 加速度 (单位: mg)
            OLED_ShowInt(oled_buffer, 0, 0, accel_display[0]);
            OLED_ShowInt(oled_buffer, 0, 3, accel_display[1]);
            OLED_ShowInt(oled_buffer, 0, 6, accel_display[2]);
            break;
        case 1: // 角速度 (单位: 0.1°/s)
            OLED_ShowInt(oled_buffer, 0, 0, gyro_display[0]);
            OLED_ShowInt(oled_buffer, 0, 3, gyro_display[1]);
            OLED_ShowInt(oled_buffer, 0, 6, gyro_display[2]);
            break;
        case 2: // 姿态角 (单位: °)
            OLED_ShowInt(oled_buffer, 0, 0, attitude_display[0]);
            OLED_ShowInt(oled_buffer, 0, 3, attitude_display[1]);
            OLED_ShowInt(oled_buffer, 0, 6, attitude_display[2]);
            break;
    }
}


/*--------------------------任务调度器---------------------------------*/
void TryNextTransfer(void)
{
	static uint8_t OLED_Just_Done = 0;
	
	if(OLED_Update_Request && !OLED_Just_Done)
	{
		OLED_Update_Request = 0;
		OLED_DrawPage();   // 根据当前页面填充缓冲
		OLED_UpdateScreen(oled_buffer, TryNextTransfer);
		OLED_Just_Done = 1;
	}
	else if(MPU6500_Read_Request)
	{
		MPU6500_ReadAll(mpu_buf, MPU_DataReady);
		OLED_Just_Done = 0;
	}
	//否则空闲
}

/*-------------------------- main ----------------------------------*/
int main(void)
{
	SysTick_Config(72000);
	TIM2_Init();
	LED_Init();
	Key_Init();
	My_I2C_Init();
	
	
	OLED_Init(OLED_InitDone);   // 启动初始化链
	
	
	while (1)
	{
		if(Systick_Tick - Key_Lasttime_Tick >= 1)
		{
			Key_Lasttime_Tick = Systick_Tick;
			Key_Task();
		}
		
		if(Systick_Tick - OLED_Lasttime_Tick >= 100)
		{
			OLED_Lasttime_Tick = Systick_Tick;
			OLED_Update_Request = 1;
		}
	}
}



/*---------------------中断服务函数--------------------*/
//Systick中断服务函数
void SysTick_Handler(void)
{
	Systick_Tick ++;
}

//TIM2中断服务函数
void TIM2_IRQHandler(void)
{
	if(TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF;
		My_I2C_Tick_Handler();
	}
	
}



//*---------------------业务任务实现--------------------*/
void Key_Task(void)
{
	uint8_t event = Key_Scan();
	switch (event)
	{
		case KEY_EVENT_SHORT:
			display_page = (display_page + 1) % 3;  // 循环切换 0->1->2->0
		break;
		
		case KEY_EVENT_LONG:
			clear_request = 1;   // 置位清零标志，在 MPU_DataReady 中处理
		break;
		
		case KEY_EVENT_NONE:
			default:
				break;
	}
}
