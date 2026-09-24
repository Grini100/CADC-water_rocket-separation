#ifndef __MPU6050_H
#define __MPU6050_H

/*typedef struct
{
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	int16_t Temp;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
}MPU6050DataTypedef;

MPU6050DataTypedef MPU6050_GetData(void);
*/
// 初始化并自检：返回 1 = WHO_AM_I 通过；0 = 自检失败（上层须禁止进入飞行流程）
uint8_t MPU6050_Init(void);
uint8_t MPU6050_ReadReg(uint8_t RegAddress, uint8_t *Data);
uint8_t MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Count);
uint8_t MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data);
uint8_t MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
									int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ);

uint8_t MPU6050_GetID(void);

#endif
