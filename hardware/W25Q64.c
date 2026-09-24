#include "stm32f10x.h"                  // Device header
#include "MySPI.h"
#include "W25Q64_Ins.h"
#include <stdint.h>

void W25Q64_Init(void)
{
	MySPI_Init();
}

void W25Q64_WaitBusy(void)//等待W25Q64忙，忙是指数据正在从缓存区转入寄存区
{
	// 超时40000次：SPI速率562.5kHz时单次循环约35us，总计约1.4s，
	// 覆盖扇区擦除最坏400ms，防止擦除未完成就提前返回导致后续命令被芯片忽略
	uint32_t Timeout=40000;
	MySPI_Start();
	MySPI_StawByte(W25Q64_Read_Status_Register_1);
	// 注意：必须用按位与&取WIP位；原代码误写为逻辑与&&，WEL等位非零时也会误判为忙
	while((MySPI_StawByte(W25Q64_DUMMY_BYTE)&0x01)==0x01)
	{
		Timeout--;
		if(Timeout==0)
		{
			break;
		}
	}
	MySPI_Stop();
}
//读取W25Q64的ID，厂商8bitID和设备16bitID（前八位表示储存器类型后八位表示容量）
void W25Q64_ReadID(uint8_t *MID,uint16_t *DID)
{
	MySPI_Start();
	MySPI_StawByte(W25Q64_JEDEC_ID);
	*MID=MySPI_StawByte(W25Q64_DUMMY_BYTE);//这里虽然用到了交换函数，但是不需要等待忙，因为设备ID储存在只读区域
	*DID=MySPI_StawByte(W25Q64_DUMMY_BYTE);
	*DID<<=8;
	*DID|=MySPI_StawByte(W25Q64_DUMMY_BYTE);
	MySPI_Stop();
}

void W25Q64_WriteEnable(void)//写使能，每次写操作之前都需要执行此函数
{
	MySPI_Start();
	MySPI_StawByte(W25Q64_Write_Enable);
	MySPI_Stop();
}
void W25Q64_PageProgram(uint32_t Address,uint8_t *DataArry,uint32_t Counts)//页编程，对指定页进行写入操作
{ 
	W25Q64_WaitBusy();
	W25Q64_WriteEnable();
	uint16_t i;
	MySPI_Start();
	MySPI_StawByte(W25Q64_Page_Program);
	MySPI_StawByte(Address>>16);
	MySPI_StawByte(Address>>8);
	MySPI_StawByte(Address);
	for(i=0;i<Counts;i++)
	{
		MySPI_StawByte(DataArry[i]);
	}
	MySPI_Stop();
}

void W25Q64_SectorErase(uint32_t Address)//扇区擦除，擦除指定扇区，这里的扇区是32位的
{ 
	W25Q64_WaitBusy();
	W25Q64_WriteEnable();
	MySPI_Start();
	MySPI_StawByte(W25Q64_Sector_Erase);
	MySPI_StawByte(Address>>16);
	MySPI_StawByte(Address>>8);
	MySPI_StawByte(Address);
	MySPI_Stop();
}

void W25Q64_BlockErase(uint32_t Address)//块擦除，擦除指定块
{ 
	W25Q64_WaitBusy();
	W25Q64_WriteEnable();
	MySPI_Start();
	MySPI_StawByte(W25Q64_Block_Erase_32);
	MySPI_StawByte(Address>>16);
	MySPI_StawByte(Address>>8);
	MySPI_StawByte(Address);
	MySPI_Stop();
}

void W25Q64_ChipErase(void)//片擦除
{ 
	W25Q64_WaitBusy();
	W25Q64_WriteEnable();
	MySPI_Start();
	MySPI_StawByte(W25Q64_Chip_Erase);
	MySPI_Stop();
}

void W25Q64_ReadData(uint32_t Address,uint8_t *DataArry,uint32_t Counts)//读取指定地址的数据
{ 
	W25Q64_WaitBusy();
	uint32_t i;
	MySPI_Start();
	MySPI_StawByte(W25Q64_Read_Data);
	MySPI_StawByte(Address>>16);
	MySPI_StawByte(Address>>8);
	MySPI_StawByte(Address);
	for(i=0;i<Counts;i++)
	{
		DataArry[i]=MySPI_StawByte(W25Q64_DUMMY_BYTE);
	}
	MySPI_Stop();
}
