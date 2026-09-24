#include "stm32f10x.h"                  // Device header
#include "AppTimer.h"

/* ============================================================
 * AppTimer：TIM4 产生 1ms 系统时基
 * 72MHz / PSC(72) = 1MHz 计数频率，计 1000 个数 = 1ms 更新中断
 * ISR 内只做 TickCnt++，绝不放任何 I2C/SPI/延时操作
 * ============================================================ */

static volatile uint32_t TickCnt = 0;

void AppTimer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	// 中断优先级分组（全工程统一设置一次即可，重复设置无害）
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	TIM_InternalClockConfig(TIM4);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Prescaler         = 72 - 1;    // 72MHz/72 = 1MHz
	TIM_TimeBaseInitStructure.TIM_Period            = 1000 - 1;  // 1MHz计1000次 = 1ms
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

	TIM_ClearFlag(TIM4, TIM_FLAG_Update);       // 清掉初始化期间可能置上的更新标志，防止一使能就误进中断
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM4, ENABLE);
}

// 获取系统毫秒时基（上电起累计，约49.7天回绕一次）
// uint32_t 对齐读写在 Cortex-M3 上是原子的，多字节共享无需关中断
uint32_t AppTimer_GetTick(void)
{
	return TickCnt;
}

// TIM4更新中断：只维护时基，保持极短
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TickCnt ++;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}
