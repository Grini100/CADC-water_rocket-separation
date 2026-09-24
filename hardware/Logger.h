#ifndef __LOGGER_H
#define __LOGGER_H

void Logger_Prepare(void);      // 阻塞擦除日志区（典型约3s），上电调用一次，OLED显示进度
void Logger_Start(void);        // 武装时调用：进入待命，检测到起飞后才开始真正记录
void Logger_Task(void);         // 调度任务：10ms一次（100Hz）
uint32_t Logger_GetCount(void); // 已写入条数（OLED显示/落盘确认用）
void Logger_Dump(void);         // 落地后调用：按"包头+条目流"把日志经串口发出（阻塞式）

#endif
