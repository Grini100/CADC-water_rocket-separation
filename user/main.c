#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "LED.h"
#include "key.h"
#include "OLED.h"
#include "MPU6050.h"
#include "AccFilter.h"
#include "Separation.h"
#include "Warmup.h"
#include "Logger.h"
#include "Serial.h"
#include "AppTimer.h"
#include "AppScheduler.h"

/* ============================ 调度任务定义 ============================ */

// IMU采样：10ms一次，含一次Burst Read，约3ms
// 必须先于 Task_Flight 注册——同周期内表序即执行序，状态机要用本拍新数据
static void Task_IMU(void)
{
	AccFilter_Update();
}

// 飞行状态机：10ms一次，只读滤波结果，微秒级
static void Task_Flight(void)
{
	Separation_Loop();
}

// 黑匣子：10ms一次。检测到起飞才开始真正写flash；页编程约0.4ms/80ms一次
// 注册在 Task_Flight 之后，保证每条记录携带的是本拍刚更新的状态
static void Task_Logger(void)
{
	Logger_Task();
}

// OLED调试显示：100ms一次，软件I2C约2ms
static void Task_OLED(void)
{
	static uint8_t LastState = 0xFF;     // 上一拍状态（初值非法，保证第一拍必走分支）
	uint8_t State = Separation_GetState();

	/* ------------ 方案3：DONE 后停刷实时数据，避免覆盖 Task_Dump 写的 DUMP 界面 ------------
	 * 根因回顾：Task_Dump 内 Logger_Dump() 阻塞几秒~几十秒，期间调度器的 Last_OLED
	 * 停留在进入 dump 之前；dump 返回的下一轮 AppScheduler_Run 立即判定 OLED 严重
	 * 过期(Now-Last>>100ms)，立刻无条件重写 4 行 → 覆盖 DUMP.../DUMP OK。
	 * 修复：进入 DONE 后 Task_OLED 永久停刷。同时在"首次刚进入 DONE"那一拍
	 * 写一行 KEY2 回读提示作为落地后的人机界面。                                         */
	if (State == SEP_STATE_DONE)
	{
		if (LastState != SEP_STATE_DONE)    // 仅刚进入 DONE 的第一拍执行一次
		{
			LastState = SEP_STATE_DONE;
			OLED_Clear();
			OLED_ShowString(1, 1, "FLIGHT DONE");
			OLED_ShowNum   (2, 1, Logger_GetCount(), 5);    // 总条数（= 飞行时长 × 100）
			OLED_ShowNum   (3, 1, AccFilter_GetErrorCount(), 5);
			OLED_ShowString(4, 1, "KEY2 DUMP");
		}
		return;                              // 后续100ms拍全跳过 → 永不覆盖 dump 写的屏幕
	}
	LastState = State;

	OLED_ShowNum(1, 1, State, 1);                                          // 状态机状态
	OLED_ShowNum(2, 1, (uint32_t)(AccFilter_GetMagnitude() * 100), 4);     // |a|，静止约" 100"
	OLED_ShowNum(3, 1, AccFilter_GetErrorCount(), 5);                      // I2C失败计数，应保持0
	OLED_ShowNum(4, 1, Logger_GetCount(), 5);                              // 黑匣子已写条数
}

// 落地回读：DONE后按KEY2，黑匣子日志经串口发往上位机
// 飞行中直接return，不触碰带阻塞消抖的Key_GetNum；落地后调用Logger_Dump
// （阻塞发送最坏约23s@115200，此时飞行已结束，阻塞无碍）
static void Task_Dump(void)
{
	if (Separation_GetState() != SEP_STATE_DONE) return;
	if (Key_GetNum() != 2) return;

	LED2_OFF();                     // 已落地，撤掉武装指示
	OLED_Clear();
	OLED_ShowString(1, 1, "DUMP...");
	Logger_Dump();
	OLED_ShowString(1, 1, "DUMP OK");
}

/* ============================ 主函数 ============================ */

int main(void)
{
	LED_Init();
	OLED_Init();
	Serial_Init();                  // USART1@115200：落地后KEY2回读黑匣子用（飞行中不发一字节）
	OLED_Clear();
	OLED_ShowString(1, 1, "IMU Self-Check");

	// WHO_AM_I 自检：失败 = 芯片不在线或I2C故障，卡死在ERROR态，禁止进入飞行流程
	// （Delay_ms 阻塞只出现在初始化/报错阶段，控制循环里不再使用）
	if (MPU6050_Init() == 0)
	{
		OLED_ShowString(2, 1, "IMU ERROR!");
		while (1)
		{
			LED1_Turn();
			Delay_ms(100);
		}
	}

	AccFilter_Init();
	Separation_Init();              // 内部含 Servo_Init，上电即锁定两个舵机

	// 预热：填满滤波窗口 + 静止健康检查，通过后才启动调度器（= 才允许进入可ARM状态）
	// 失败自动重试，OLED 显示具体原因；预热期间请把箭体放稳、不要触碰
	{
		uint8_t WarmupResult;

		while ((WarmupResult = Warmup_Run()) != WARMUP_OK)
		{
			OLED_ShowString(2, 1, "FAIL:");
			if      (WarmupResult == WARMUP_ERR_I2C)  OLED_ShowString(2, 6, "I2C ");
			else if (WarmupResult == WARMUP_ERR_MEAN) OLED_ShowString(2, 6, "MEAN");
			else                                      OLED_ShowString(2, 6, "MOVE");
			LED1_Turn();
			Delay_ms(1000);         // 停留1秒让失败原因可读，然后自动重试
		}
	}
	LED1_OFF();                     // 预热通过：灯灭作为指示

	Logger_Prepare();               // 擦除黑匣子日志区（约3s，OLED显示进度）

	/* ================= ARM 等待：结构性安全门 =================
	 * 按下 KEY1 之前，调度器/状态机尚未启动——任何磕碰冲击都不可能
	 * 触发射出舵机。阻塞轮询仅存在于武装前（初始化阶段允许阻塞），
	 * 武装后不再扫描按键，任务里没有长阻塞。
	 * 发射场流程建议：上电(放稳) → 自检+预热 → 擦日志 → 装配检查
	 *               → 立箭上架 → 按KEY1武装 → 打气 → 发射
	 * ================= ARM 等待：结构性安全门 ================= */
	Key_Init();
	OLED_Clear();
	OLED_ShowString(1, 1, "IMU OK");
	OLED_ShowString(2, 1, "PRESS KEY1 ARM");
	while (Key_GetNum() != 1)       // 1 = KEY1(PB1)；KEY2(PB0) 暂保留
	{
		AccFilter_Update();         // 武装前保持采样，OLED可实时确认传感器活着
		OLED_ShowNum(3, 1, (uint32_t)(AccFilter_GetMagnitude() * 100), 4);
		Delay_ms(10);
	}

	LED2_ON();                      // 已武装的物理指示（PB5外部LED）
	Logger_Start();                 // 黑匣子进入待命，起飞后自动记录
	OLED_Clear();

	AppTimer_Init();                // 从此刻起状态机才"存在"
	AppScheduler_Init();
	AppScheduler_AddTask(10,  Task_IMU);      // 表序=执行序：IMU在前
	AppScheduler_AddTask(10,  Task_Flight);
	AppScheduler_AddTask(10,  Task_Logger);   // 在Flight之后：记录本拍刚更新的状态
	AppScheduler_AddTask(100, Task_OLED);
	AppScheduler_AddTask(100, Task_Dump);     // 落地后按KEY2回读日志；飞行中空转返回

	while (1)
	{
		AppScheduler_Run();         // 主循环只做一件事：协作式调度
	}
}
