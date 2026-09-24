#ifndef __APPSCHEDULER_H
#define __APPSCHEDULER_H

void    AppScheduler_Init(void);
uint8_t AppScheduler_AddTask(uint16_t Period_ms, void (*Task)(void));
void    AppScheduler_Run(void);

#endif
