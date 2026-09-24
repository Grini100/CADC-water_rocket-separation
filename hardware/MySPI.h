#ifndef __MYSPI_H
#define __MYSPI_H

void MySPI_W_SS(uint8_t BitValue);
void MySPI_Init(void);
void MySPI_Start(void);
void MySPI_Stop(void);
uint8_t MySPI_StawByte(uint8_t ByteSend);



#endif
