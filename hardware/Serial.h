#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

extern uint8_t Serial_TxPacket[];//外部数组可以不写元素个数
extern uint8_t Serial_RxPacket[];

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array,uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Num,uint8_t Length);
void Serial_Print(char *format,...);
void Serial_SendPacket(void);

uint8_t Serial_GetRxFlag(void);



#endif
