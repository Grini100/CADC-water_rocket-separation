#include "stm32f10x.h"                  // Device header
#include "MPU6050_Reg.h"
#include "MPU6050.h"
#include "Delay.h"


#define MPU6050_Address     0xD0

// 等待I2C事件：返回1=事件到达，0=超时失败
// 超时上限2000次：正常传输的事件在数百us内必然到达，余量约10倍；
// 总线彻底故障时单次等待最多阻塞约1~2ms，不会把10ms主循环节拍拖垮
static uint8_t MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout = 2000;
	while(I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if(Timeout == 0)
		{
			return 0;					// 超时：通信异常
		}
	}
	return 1;							// 事件正常到达
}

// 写寄存器：返回1=成功，0=失败
// 任一环节超时即失败，并在Fail处尝试发STOP释放总线，给下一次传输留恢复机会
uint8_t MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)
{
	I2C_GenerateSTART(I2C2,ENABLE);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT))               goto Fail;

	I2C_Send7bitAddress(I2C2,MPU6050_Address,I2C_Direction_Transmitter);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto Fail;

	I2C_SendData(I2C2,RegAddress);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTING))         goto Fail;

	I2C_SendData(I2C2,Data);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED))          goto Fail;

	I2C_GenerateSTOP(I2C2,ENABLE);
	return 1;

Fail:
	I2C_GenerateSTOP(I2C2,ENABLE);		// 释放总线
	return 0;
}

// 读单字节：返回1=成功，0=失败
uint8_t MPU6050_ReadReg(uint8_t RegAddress, uint8_t *Data)
{
	I2C_GenerateSTART(I2C2,ENABLE);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT))               goto Fail;

	I2C_Send7bitAddress(I2C2,MPU6050_Address,I2C_Direction_Transmitter);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto Fail;

	I2C_SendData(I2C2,RegAddress);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED))          goto Fail;

	I2C_GenerateSTART(I2C2,ENABLE);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT))               goto Fail;

	I2C_Send7bitAddress(I2C2,MPU6050_Address,I2C_Direction_Receiver);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))    goto Fail;

	I2C_AcknowledgeConfig(I2C2,DISABLE);
	I2C_GenerateSTOP(I2C2,ENABLE);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED))             goto Fail;

	*Data=I2C_ReceiveData(I2C2);
	I2C_AcknowledgeConfig(I2C2,ENABLE);
	return 1;

Fail:
	I2C_GenerateSTOP(I2C2,ENABLE);		// 释放总线
	I2C_AcknowledgeConfig(I2C2,ENABLE);	// 恢复ACK默认态
	return 0;
}

// 连续读取多个寄存器（Burst Read）：返回1=成功，0=失败
// 注意：Count 必须 >= 2；单字节读取请用 MPU6050_ReadReg
uint8_t MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *Data, uint8_t Count)
{
	uint8_t i;

	I2C_GenerateSTART(I2C2,ENABLE);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT))               goto Fail;

	I2C_Send7bitAddress(I2C2,MPU6050_Address,I2C_Direction_Transmitter);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) goto Fail;

	I2C_SendData(I2C2,RegAddress);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED))          goto Fail;

	I2C_GenerateSTART(I2C2,ENABLE);			// 重复起始条件：不发STOP直接转为读方向
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT))               goto Fail;

	I2C_Send7bitAddress(I2C2,MPU6050_Address,I2C_Direction_Receiver);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))    goto Fail;

	for(i = 0; i < Count - 1; i ++)			// 前 Count-1 字节：应答ACK，从机继续发
	{
		if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED))         goto Fail;
		Data[i]=I2C_ReceiveData(I2C2);
	}

	I2C_AcknowledgeConfig(I2C2,DISABLE);	// 最后一字节前：非应答NACK告知从机结束
	I2C_GenerateSTOP(I2C2,ENABLE);
	if(!MPU6050_WaitEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED))             goto Fail;

	Data[Count-1]=I2C_ReceiveData(I2C2);
	I2C_AcknowledgeConfig(I2C2,ENABLE);		// 恢复ACK，不影响下一次传输
	return 1;

Fail:
	I2C_GenerateSTOP(I2C2,ENABLE);			// 释放总线
	I2C_AcknowledgeConfig(I2C2,ENABLE);		// 恢复ACK默认态
	return 0;
}

// 初始化并自检
// 返回值：1 = 寄存器配置全部成功且 WHO_AM_I 校验通过；0 = 任一环节失败，上层应禁止进入飞行流程
uint8_t MPU6050_Init(void)
{
	uint8_t i;

	Delay_ms(50);							// 等待MPU6050上电启动完成（VDD稳定后芯片需数十ms就绪）

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);

	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed=50000;
	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1=0x00;
	I2C_Init(I2C2,&I2C_InitStructure);

	I2C_Cmd(I2C2,ENABLE);

	// 配置寄存器：任一写入失败都判为自检不通过
	if(!MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01)) return 0;	// 退出睡眠，时钟源选X轴陀螺PLL
	if(!MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00)) return 0;	// 六轴全部使能
	if(!MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x09)) return 0;	// 采样率 1kHz/(1+9)=100Hz，与主循环节拍一致
	// DLPF配置0x03：内部低通带宽44Hz、延迟约4.9ms
	// 旧值0x06带宽仅5Hz、延迟19ms，对起飞/失重检测响应太慢；主滤波交由软件滑动平均
	// 台架验证标准：静止时|a|波动峰峰值<0.05g；不达标可退回0x04（20Hz/8.5ms）
	if(!MPU6050_WriteReg(MPU6050_CONFIG,0x03))     return 0;
	if(!MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x18))return 0;	// 陀螺仪满量程±2000deg/s
	if(!MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x18))return 0;	// 加速度计满量程±16g，灵敏度2048 LSB/g

	// WHO_AM_I 自检：预期0x68（MPU6050），重试5次规避上电瞬间I2C偶发不稳定
	for(i = 0; i < 5; i ++)
	{
		if(MPU6050_GetID() == 0x68)
		{
			return 1;						// 自检通过
		}
		Delay_ms(10);
	}
	return 0;									// 芯片不在线或I2C故障
}

// 读取WHO_AM_I：正常返回器件ID（MPU6050为0x68），读取失败返回0xFF（不会与0x68混淆）
uint8_t MPU6050_GetID(void)
{
	uint8_t id = 0xFF;
	MPU6050_ReadReg(MPU6050_WHO_AM_I, &id);
	return id;
}

// 读取六轴原始数据：返回1=成功，0=失败（残缺数据不可用，上层须冻结旧值）
uint8_t MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,
									int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ)
{
	uint8_t Data[14];

	// 从 0x3B(ACCEL_XOUT_H) 起一次 Burst Read 连续读出 Ax Ay Az Temp Gx Gy Gz 共14字节
	// 相比逐字节读取：12次I2C事务合并为1次，50kHz总线下耗时约 13ms -> 3ms
	if(!MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14))
	{
		return 0;
	}

	*AccX =(int16_t)((Data[0] << 8) | Data[1]);
	*AccY =(int16_t)((Data[2] << 8) | Data[3]);
	*AccZ =(int16_t)((Data[4] << 8) | Data[5]);
	// Data[6]/Data[7] 为温度，本模块暂不使用
	*GyroX=(int16_t)((Data[8] << 8) | Data[9]);
	*GyroY=(int16_t)((Data[10] << 8) | Data[11]);
	*GyroZ=(int16_t)((Data[12] << 8) | Data[13]);

	return 1;
}
