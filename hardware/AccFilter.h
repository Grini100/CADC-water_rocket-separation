#ifndef __ACCFILTER_H
#define __ACCFILTER_H

void AccFilter_Init(void);
uint8_t AccFilter_Update(void);          // 返回1=采样成功，0=失败（滤波器已冻结）

float AccFilter_GetMagnitude(void);
uint16_t AccFilter_GetErrorCount(void);  // I2C失败累计，地面应为0
void AccFilter_ClearErrorCount(void);
uint8_t AccFilter_GetLastOk(void);       // 最近一次采样是否成功
void AccFilter_GetRaw(int16_t *Ax, int16_t *Ay, int16_t *Az,
					  int16_t *Gx, int16_t *Gy, int16_t *Gz);
float AccFilter_GetAxisX(void);
float AccFilter_GetAxisY(void);
float AccFilter_GetAxisZ(void);

#endif
