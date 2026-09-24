#ifndef __WARMUP_H
#define __WARMUP_H

#include "stm32f10x.h"

// Warmup_Run 返回值
#define WARMUP_OK        0      // 预热通过：窗口已填满真实数据且静止检查合格
#define WARMUP_ERR_I2C   1      // 预热期间出现I2C读取失败
#define WARMUP_ERR_MEAN  2      // |a|均值异常：传感器疑似故障
#define WARMUP_ERR_MOVE  3      // |a|波动过大：预热期间有人在晃动箭体

uint8_t Warmup_Run(void);

#endif
