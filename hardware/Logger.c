#include "stm32f10x.h"                  // Device header
#include "Logger.h"
#include "W25Q64.h"
#include "AppTimer.h"
#include "AccFilter.h"
#include "Separation.h"
#include "OLED.h"
#include "Serial.h"

/* ============================================================
 * Logger：飞行数据黑匣子（W25Q64）
 *
 * 记录策略：
 *   - 武装(Logger_Start)后进入待命，检测到状态离开IDLE（起飞）才开始记录
 *   - 100Hz写入，到DONE（开伞完成）自动停止，容量自动截断
 *   - RAM页缓冲攒满256B(8条)才页编程，飞行中绝不擦除、绝不满页跨写
 *
 * 存储布局（显式按字节打包，不依赖结构体对齐）：
 *   64扇区 × 4KB = 256KB ÷ 32B/条 = 8192条 = 81.9s @100Hz
 *   页 = 256B = 8条，写地址始终与页对齐
 *
 * 条目格式（32B，小端）：
 *   [0..3]  uint32 Tick      相对起飞时刻的毫秒数
 *   [4]     uint8  State     状态机状态0~4
 *   [5]     uint8  Flags     bit0=本条为I2C失败时的冻结旧值
 *   [6..7]  int16  Ax        原始LSB（±16g量程，2048 LSB/g）
 *   [8..9]  int16  Ay
 *   [10..11]int16  Az
 *   [12..13]int16  Gx        原始LSB（±2000deg/s）
 *   [14..15]int16  Gy
 *   [16..17]int16  Gz
 *   [18..19]uint16 Mag100    |a|×100
 *   [20..31]0xFF 保留
 * ============================================================ */

#define LOG_BASE_ADDR     0x000000UL
#define LOG_SECTOR_SIZE   4096UL
#define LOG_SECTOR_NUM    64UL
#define LOG_ENTRY_SIZE    32UL
#define LOG_PAGE_SIZE     256UL
#define LOG_PAGE_ENTRIES  (LOG_PAGE_SIZE / LOG_ENTRY_SIZE)                    // 8
#define LOG_MAX_ENTRIES   (LOG_SECTOR_NUM * LOG_SECTOR_SIZE / LOG_ENTRY_SIZE) // 8192

static uint8_t  PageBuf[LOG_PAGE_SIZE]; // RAM页缓冲
static uint32_t WriteIdx;               // 已完整写入flash的条数
static uint8_t  PageFill;               // 当前页缓冲内已缓存条数
static uint8_t  Running;                // 记录进行中
static uint8_t  Pending;                // 已武装、等起飞
static uint32_t StartTick;              // 起飞时刻的时基

// 上电调用：擦除日志区。流水线式逐扇区擦除，第N次调用的WaitBusy
// 会顺带等完第N-1次擦除，总耗时≈扇区数×单次擦除时间(典型45ms)
void Logger_Prepare(void)
{
	uint32_t s;
	uint8_t dummy;

	W25Q64_Init();

	OLED_ShowString(1, 1, "ERASE LOG");
	for(s = 0; s < LOG_SECTOR_NUM; s ++)
	{
		W25Q64_SectorErase(LOG_BASE_ADDR + s * LOG_SECTOR_SIZE);
		if((s % 8) == 0)
		{
			OLED_ShowNum(1, 10, (uint32_t)(s * 100 / LOG_SECTOR_NUM), 3);  // 进度%
		}
	}
	W25Q64_ReadData(LOG_BASE_ADDR, &dummy, 1);  // 等最后一次擦除彻底完成

	Running = 0;
	Pending  = 0;
	WriteIdx = 0;
	PageFill = 0;
	OLED_ShowString(2, 1, "ERASE OK");
}

// 武装时调用：进入待命，等起飞
void Logger_Start(void)
{
	WriteIdx = 0;
	PageFill = 0;
	Pending  = 1;
	Running  = 1;
}

// 停止：残留不满一页的数据补0xFF后落盘
static void Logger_Stop(void)
{
	uint16_t i;

	if(PageFill > 0)
	{
		for(i = PageFill * LOG_ENTRY_SIZE; i < LOG_PAGE_SIZE; i ++)
		{
			PageBuf[i] = 0xFF;
		}
		W25Q64_PageProgram(LOG_BASE_ADDR + (WriteIdx / LOG_PAGE_ENTRIES) * LOG_PAGE_SIZE,
						   PageBuf, LOG_PAGE_SIZE);
		WriteIdx += PageFill;
		PageFill = 0;
	}
	Running = 0;
}

// 调度任务：10ms一次
void Logger_Task(void)
{
	uint8_t *p;
	uint32_t Tick;
	uint16_t Mag100;
	int16_t Ax, Ay, Az, Gx, Gy, Gz;
	uint8_t i;

	if(!Running) return;

	// 待命期：等状态离开IDLE（起飞确认ASCENT）才开始记录
	if(Pending)
	{
		if(Separation_GetState() == SEP_STATE_IDLE) return;
		Pending  = 0;
		StartTick = AppTimer_GetTick();
	}

	// 容量到顶：强制收尾
	if(WriteIdx >= LOG_MAX_ENTRIES)
	{
		Logger_Stop();
		return;
	}

	// ---- 组装一条32B记录（显式字节打包，小端） ----
	Tick = AppTimer_GetTick() - StartTick;
	p = &PageBuf[PageFill * LOG_ENTRY_SIZE];

	p[0] = (uint8_t)(Tick);
	p[1] = (uint8_t)(Tick >> 8);
	p[2] = (uint8_t)(Tick >> 16);
	p[3] = (uint8_t)(Tick >> 24);
	p[4] = (uint8_t)Separation_GetState();
	p[5] = AccFilter_GetLastOk() ? 0x00 : 0x01;   // bit0: 冻结旧值标志

	AccFilter_GetRaw(&Ax, &Ay, &Az, &Gx, &Gy, &Gz);
	p[6]  = (uint8_t)Ax;  p[7]  = (uint8_t)(Ax >> 8);
	p[8]  = (uint8_t)Ay;  p[9]  = (uint8_t)(Ay >> 8);
	p[10] = (uint8_t)Az;  p[11] = (uint8_t)(Az >> 8);
	p[12] = (uint8_t)Gx;  p[13] = (uint8_t)(Gx >> 8);
	p[14] = (uint8_t)Gy;  p[15] = (uint8_t)(Gy >> 8);
	p[16] = (uint8_t)Gz;  p[17] = (uint8_t)(Gz >> 8);

	Mag100 = (uint16_t)(AccFilter_GetMagnitude() * 100.0f);
	p[18] = (uint8_t)Mag100;
	p[19] = (uint8_t)(Mag100 >> 8);

	for(i = 20; i < LOG_ENTRY_SIZE; i ++) p[i] = 0xFF;

	// ---- 攒满一页落盘 ----
	PageFill ++;
	if(PageFill >= LOG_PAGE_ENTRIES)
	{
		// PageProgram内部的WaitBusy会确认上一次编程已完成（80ms远大于编程时间）
		W25Q64_PageProgram(LOG_BASE_ADDR + (WriteIdx / LOG_PAGE_ENTRIES) * LOG_PAGE_SIZE,
						   PageBuf, LOG_PAGE_SIZE);
		WriteIdx += LOG_PAGE_ENTRIES;
		PageFill = 0;
	}

	// DONE（开伞完成）：本条已记完，收尾
	if(Separation_GetState() == SEP_STATE_DONE)
	{
		Logger_Stop();
	}
}

// 已写入条数
uint32_t Logger_GetCount(void)
{
	return WriteIdx + PageFill;
}

// 落地后导出：按"包头+条目流"把日志从串口发出（DONE后调用一次）
// 协议：ASCII包头行 "WLOG,<4位条数>\n"，随后紧接 <条数>×32B 原始条目流
// 上位机Python：按行读到WLOG包头取条数，再定长读二进制即可
// 最坏8192条×32B=256KB @115200约23s发完；阻塞式发送，期间仅状态机空转，落地后安全
void Logger_Dump(void)
{
	uint8_t Buf[LOG_PAGE_SIZE];
	uint32_t Sent;
	uint32_t n;

	Serial_SendString("WLOG,");
	Serial_SendNumber(WriteIdx, 4);
	Serial_SendString("\n");

	Sent = 0;
	while(Sent < WriteIdx)
	{
		n = WriteIdx - Sent;
		if(n > LOG_PAGE_ENTRIES) n = LOG_PAGE_ENTRIES;

		// 每次读一整页（256B），只发有效条目部分（尾部补位0xFF不外发）
		W25Q64_ReadData(LOG_BASE_ADDR + (Sent / LOG_PAGE_ENTRIES) * LOG_PAGE_SIZE,
						Buf, LOG_PAGE_SIZE);
		Serial_SendArray(Buf, (uint16_t)(n * LOG_ENTRY_SIZE));
		Sent += n;
	}
}
