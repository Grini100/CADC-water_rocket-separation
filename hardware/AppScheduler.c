#include "stm32f10x.h"                  // Device header
#include "AppScheduler.h"
#include "AppTimer.h"

/* ============================================================
 * AppScheduler：协作式毫秒调度器（非抢占）
 *
 * 纪律：每个任务必须短！当前任务预算：
 *   Task_IMU    约3ms   （Burst Read，唯一的长动作，预算内）
 *   Task_Flight 微秒级  （状态机判定）
 *   Task_OLED   约2ms   （软件I2C显示，100ms一次）
 * 若未来加入更耗时的任务（如flash页编程约3ms），仍可接受；
 * 但绝不允许在任务里放秒级阻塞（如扇区擦除）——那会丢IMU采样。
 * ============================================================ */

#define SCHED_MAX_TASKS   8

typedef struct
{
	uint16_t Period;        // 调度周期（ms）
	uint32_t Last;          // 上次执行时刻（ms时基）
	void (*Task)(void);     // 任务函数
}SchedItem_t;

static SchedItem_t Items[SCHED_MAX_TASKS];
static uint8_t TaskNum = 0;

void AppScheduler_Init(void)
{
	TaskNum = 0;
}

// 注册任务：返回1=成功，0=失败（表满/空指针/周期为0）
// 注意：同一周期内先注册的先执行——Task_IMU 必须先于 Task_Flight 注册
uint8_t AppScheduler_AddTask(uint16_t Period_ms, void (*Task)(void))
{
	if(TaskNum >= SCHED_MAX_TASKS || Task == 0 || Period_ms == 0)
	{
		return 0;
	}

	Items[TaskNum].Period = Period_ms;
	Items[TaskNum].Last   = AppTimer_GetTick();	// 从注册时刻起算，避免注册前积压
	Items[TaskNum].Task   = Task;
	TaskNum ++;
	return 1;
}

// 在主循环中反复调用：检查并执行到期任务
void AppScheduler_Run(void)
{
	uint8_t i;
	uint32_t Now;

	for(i = 0; i < TaskNum; i ++)
	{
		Now = AppTimer_GetTick();				// 每个任务前重读时基：前面任务阻塞过，后面判断仍准确
		if(Now - Items[i].Last >= Items[i].Period)	// 无符号减法，时基回绕依然正确
		{
			Items[i].Last = Now;				// 到期即刷新：阻塞恢复后顺延一拍，不突发补跑
			Items[i].Task();
		}
	}
}
