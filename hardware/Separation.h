#ifndef __SEPARATION_H
#define __SEPARATION_H

// 状态机当前状态（可在 main 中读出来用于 OLED 调试显示）
typedef enum
{
	SEP_STATE_IDLE     = 0,   // 待发（静止检测中，不使能任何触发）
	SEP_STATE_ASCENT   = 1,   // 确认已起飞，动力段（等待加速度回落）
	SEP_STATE_COAST    = 2,   // 已确认动力耗尽，滑行中（等待失重）
	SEP_STATE_SEP_DONE = 3,   // 已分离，等待延时开伞
	SEP_STATE_DONE     = 4    // 已开伞，流程结束
}SepState_t;

void      Separation_Init(void);
void      Separation_Loop(void);
SepState_t Separation_GetState(void);

#endif
