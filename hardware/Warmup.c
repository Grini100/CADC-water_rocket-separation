#include "stm32f10x.h"                  // Device header
#include "Warmup.h"
#include "AccFilter.h"
#include "OLED.h"
#include "Delay.h"

/* ============================================================
 * Warmup：上电预热
 * 目的1：用真实数据填满滑动平均窗口，消除"0.125g→1g"的假启动过渡
 * 目的2：利用静止段做健康检查，通过后才允许系统进入可ARM状态
 *
 * 流程（阻塞约2.6s，仅上电调用，符合"初始化阶段允许阻塞"的纪律）：
 *   前 80ms  ：填窗段，8个采样填满窗口，数据不参与统计
 *   后 1920ms：统计段，记录 |a| 的均值与峰峰值
 *
 * 判据（三条全过才返回 WARMUP_OK，否则返回具体原因，由 main 自动重试）：
 *   1. 统计段无I2C失败    —— 冻结值会造成"假平稳"，必须为零
 *   2. 均值 0.5~1.5g      —— 静止时 |a|≈1g 与姿态无关（倾斜放也算静止）
 *   3. 峰峰值 <= 0.2g     —— 超过说明有人在晃动/搬运箭体
 * ============================================================ */

#define WARMUP_FILL_NUM     8                       // 填窗采样数（=滤波窗口宽度）
#define WARMUP_STAT_NUM     192                     // 统计采样数（约1.9s）
#define WARMUP_TOTAL_NUM    (WARMUP_FILL_NUM + WARMUP_STAT_NUM)

#define WARMUP_MEAN_LOW     0.5f                    // 均值判据下限
#define WARMUP_MEAN_HIGH    1.5f                    // 均值判据上限
#define WARMUP_RANGE_MAX    0.2f                    // 峰峰值判据上限

uint8_t Warmup_Run(void)
{
	uint16_t i;
	float mag;
	float Sum = 0.0f;
	float Min = 10.0f;
	float Max = -10.0f;

	AccFilter_ClearErrorCount();					// 重试前清零失败计数，否则旧账会冤枉本次

	for(i = 0; i < WARMUP_TOTAL_NUM; i ++)
	{
		AccFilter_Update();
		mag = AccFilter_GetMagnitude();

		if(i >= WARMUP_FILL_NUM)					// 窗口已满，之后的数据才可信
		{
			Sum += mag;
			if(mag < Min) Min = mag;
			if(mag > Max) Max = mag;
		}

		if((i % 25) == 0)							// 每250ms刷一次进度（OLED约1ms，可接受）
		{
			OLED_ShowString(1, 1, "WARMUP");
			OLED_ShowNum(1, 8, (i + 1) / 2, 3);		// 进度百分比（总200次）
		}

		Delay_ms(10);								// 与调度节拍一致的100Hz采样
	}

	// 判据1：统计段I2C零失败
	if(AccFilter_GetErrorCount() != 0)
	{
		return WARMUP_ERR_I2C;
	}

	// 判据2：均值在合理范围（说明传感器活着且读数物理上合理）
	if(Sum / WARMUP_STAT_NUM < WARMUP_MEAN_LOW || Sum / WARMUP_STAT_NUM > WARMUP_MEAN_HIGH)
	{
		return WARMUP_ERR_MEAN;
	}

	// 判据3：峰峰值足够小（说明确实静止没人碰）
	if(Max - Min > WARMUP_RANGE_MAX)
	{
		return WARMUP_ERR_MOVE;
	}

	return WARMUP_OK;
}
