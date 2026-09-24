#ifndef __W25Q64_H
#define __W25Q64_H

void W25Q64_Init(void);
void W25Q64_ReadID(uint8_t *MID,uint16_t *DID);
void W25Q64_PageProgram(uint32_t Address,uint8_t *DataArry,uint32_t Counts);
void W25Q64_SectorErase(uint32_t Address);
void W25Q64_BlockErase(uint32_t Address);
void W25Q64_ChipErase(void);
void W25Q64_ReadData(uint32_t Address,uint8_t *DataArry,uint32_t Counts);


#endif
