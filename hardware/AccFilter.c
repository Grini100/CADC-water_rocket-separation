#include "stm32f10x.h"                  // Device header
#include "AccFilter.h"
#include "MPU6050.h"
#include <math.h>

/* ============================ 可调参数 ============================ */
// 加速度计量程 ±16g（MPU6050_Init 中 ACCEL_CONFIG = 0x18）
// ±16g 量程下灵敏度 = 32768 / 32 = 2048 LSB/g
#define ACC_LSB_PER_G       2048.0f

// 滑动平均窗口大小：越大越平滑，但延迟越大
// 窗口 N、采样周期 T 时，群延迟约为 (N-1)/2 * T
#define FILTER_WINDOW       8

/* ============================ 内部类型 ============================ */
// 单轴滑动平均滤波器（环形缓冲 + 递推求和，每次更新只需 O(1) 运算）
typedef struct
{
	float Buf[FILTER_WINDOW];   // 环形缓冲区
	uint8_t Index;              // 当前写入位置
	float Sum;                  // 缓冲区内数据之和
}MovAvg_t;

/* ============================ 内部变量 ============================ */
static MovAvg_t AvgX, AvgY, AvgZ;       // 三轴各自的滤波器
static float AccMag = 1.0f;             // 滤波后的合加速度（单位：g），初始为1g对应静止
static uint16_t ErrorCnt = 0;           // I2C读取失败累计计数（用于健康诊断）
static uint8_t LastOk = 1;              // 最近一次采样是否成功（Logger打标志用）
static int16_t LastAx, LastAy, LastAz;  // 最近一次有效原始值（Logger记录用）
static int16_t LastGx, LastGy, LastGz;

/* ============================ 内部函数 ============================ */
static void MovAvg_Init(MovAvg_t *f)
{
	uint8_t i;
	for(i = 0; i < FILTER_WINDOW; i ++)
	{
		f->Buf[i] = 0.0f;
	}
	f->Index = 0;
	f->Sum = 0.0f;
}

static float MovAvg_Update(MovAvg_t *f, float NewVal)
{
	f->Sum -= f->Buf[f->Index];         // 减去将被覆盖的旧值
	f->Buf[f->Index] = NewVal;          // 新值写入环形缓冲
	f->Sum += NewVal;                   // 累加新值
	f->Index ++;                        // 指针前移
	if(f->Index >= FILTER_WINDOW)
	{
		f->Index = 0;
	}
	return f->Sum / FILTER_WINDOW;      // 返回窗口内平均值
}

/* ============================ 对外接口 ============================ */
// 初始化滤波器（上电后调用一次）
void AccFilter_Init(void)
{
	MovAvg_Init(&AvgX);
	MovAvg_Init(&AvgY);
	MovAvg_Init(&AvgZ);
	AccMag   = 1.0f;
	ErrorCnt = 0;
}

// 更新一次：读取 MPU6050 -> 单位换算 -> 滑动平均 -> 计算合加速度
// 建议以固定周期调用（如 10ms 一次，即 100Hz）
// 返回值：1 = 本次采样成功；0 = I2C读取失败（滤波器冻结）
uint8_t AccFilter_Update(void)
{
	int16_t RawX, RawY, RawZ;
	int16_t GyroX, GyroY, GyroZ;        // 本模块暂不使用陀螺仪数据

	// 失败冻结策略：读取失败时不向环形缓冲写入任何数据，AccMag保持上一次的值
	// 绝不能把0或残缺数据喂进滤波器——合加速度瞬间归0会被状态机误判成失重，导致假分离！
	if(!MPU6050_GetData(&RawX, &RawY, &RawZ, &GyroX, &GyroY, &GyroZ))
	{
		if(ErrorCnt < 0xFFFF) ErrorCnt ++;   // 失败计数（饱和，防溢出回绕）
		LastOk = 0;
		return 0;
	}

	LastOk = 1;
	LastAx = RawX;  LastAy = RawY;  LastAz = RawZ;   // 缓存原始值供Logger记录
	LastGx = GyroX; LastGy = GyroY; LastGz = GyroZ;

	// 原始值换算为 g 单位（±16g 量程下满量程 ±32768）
	float x = RawX / ACC_LSB_PER_G;
	float y = RawY / ACC_LSB_PER_G;
	float z = RawZ / ACC_LSB_PER_G;

	// 三轴分别滑动平均
	float fx = MovAvg_Update(&AvgX, x);
	float fy = MovAvg_Update(&AvgY, y);
	float fz = MovAvg_Update(&AvgZ, z);

	// 合加速度模长 |a| = sqrt(ax^2 + ay^2 + az^2)，单位 g
	// 与箭体姿态无关，正倒倾斜都能正确反映"受力大小"
	AccMag = sqrtf(fx * fx + fy * fy + fz * fz);

	return 1;
}

// 获取滤波后的合加速度（单位：g）
float AccFilter_GetMagnitude(void)
{
	return AccMag;
}

// 获取I2C读取失败累计计数：地面上电后应保持0，持续增长说明接线/传感器有问题
uint16_t AccFilter_GetErrorCount(void)
{
	return ErrorCnt;
}

// 清零失败计数（Warmup 重试前调用）
void AccFilter_ClearErrorCount(void)
{
	ErrorCnt = 0;
}

// 最近一次采样是否成功（1=成功，0=失败且当前值为冻结旧值）
uint8_t AccFilter_GetLastOk(void)
{
	return LastOk;
}

// 获取最近一次有效采样的六轴原始值（LSB，±16g量程下2048 LSB/g）
void AccFilter_GetRaw(int16_t *Ax, int16_t *Ay, int16_t *Az,
								  int16_t *Gx, int16_t *Gy, int16_t *Gz)
{
	*Ax = LastAx;  *Ay = LastAy;  *Az = LastAz;
	*Gx = LastGx;  *Gy = LastGy;  *Gz = LastGz;
}

// 获取单轴滤波值（单位：g），可用于 OLED 调试显示
float AccFilter_GetAxisX(void)  {return AvgX.Sum / FILTER_WINDOW;}
float AccFilter_GetAxisY(void)  {return AvgY.Sum / FILTER_WINDOW;}
float AccFilter_GetAxisZ(void)  {return AvgZ.Sum / FILTER_WINDOW;}
