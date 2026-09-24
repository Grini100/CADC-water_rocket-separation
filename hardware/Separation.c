#include "stm32f10x.h"                  // Device header
#include "Separation.h"
#include "AccFilter.h"
#include "Servo.h"
#include "LED.h"

/* ========================================== 可调参数 ==========================================
   触发参数的设计前提：主循环调用 Separation_Loop() 的周期 = 10ms（和 AccFilter_Update 同步）
   如果你们实际调用周期不是 10ms，把下面 *_MS 的参数按比例换算成计数即可。
================================================================================================ */

// IDLE → ASCENT：检测到"已起飞"的门槛
// 动力段水喷推力产生的合加速度通常在 2~5g 之间，1.8g 留出足够裕量
#define LAUNCH_THRESHOLD_G        1.8f
// 需要连续满足多少个 10ms 周期才确认起飞（抗偶然碰振动）
#define LAUNCH_CONFIRM_MS         100     // 100ms
#define LAUNCH_CONFIRM_CNT        (LAUNCH_CONFIRM_MS / 10)

// ASCENT → COAST：检测"动力已耗尽"的门槛
// 熄火后滑行段合加速度会回落到 1~1.5g（含空气阻力的"等效支撑"），设 1.6g 保守
#define COAST_THRESHOLD_G         1.6f
#define COAST_CONFIRM_MS          200     // 连续 200ms 低于该值 → 确认滑行
#define COAST_CONFIRM_CNT         (COAST_CONFIRM_MS / 10)

// COAST → SEP_DONE：失重触发条件
// 自由下落时合加速度约 0.05~0.25g（含残余风阻与测量噪声），0.3g 为经验值
#define WEIGHTLESS_THRESHOLD_G    0.3f
// 必须连续多少个 10ms 周期才"确认失重"——防止单点噪声误触发
#define WEIGHTLESS_CONFIRM_MS     300     // 300ms
#define WEIGHTLESS_CONFIRM_CNT    (WEIGHTLESS_CONFIRM_MS / 10)

// 安全保险：滑行太久还没检测到失重（如箭体横着飞、传感器故障），强制超时后仍分离
// 水火箭总上升时间一般不超过 3 秒，超时设为 3 秒
#define COAST_TIMEOUT_MS          3000
#define COAST_TIMEOUT_CNT         (COAST_TIMEOUT_MS / 10)

// SEP_DONE → DONE：分离后延时多久开伞（滑翔机先弹开、改出，再释放主伞）
#define PARACHUTE_DELAY_MS        1500
#define PARACHUTE_DELAY_CNT       (PARACHUTE_DELAY_MS / 10)

// 舵机动作角度（根据机械安装微调，单位：度）
#define SERVO_PLANE_LOCK_ANGLE    0       // 箭体与滑翔机锁合位置
#define SERVO_PLANE_RELEASE_ANGLE 90      // 释放位置
#define SERVO_UMB_LOCK_ANGLE      0       // 伞绳锁紧位置
#define SERVO_UMB_RELEASE_ANGLE   90      // 开伞位置

/* ========================================== 内部变量 ========================================== */
static SepState_t State  = SEP_STATE_IDLE;
static uint32_t   TmrCnt = 0;      // 各状态下共用的计数器（以 10ms 为单位）
static uint16_t   WeightlessCnt = 0;   // 滑行段"失重持续计数"（10ms 为单位），移至文件顶部以兼容 C89

/* ========================================== 内部函数 ========================================== */

// 执行分离动作：CH2 舵机释放滑翔机
static void DoSeparation(void)
{
	Servo_SetPlaneAngle(SERVO_PLANE_RELEASE_ANGLE);
	LED1_ON();                                  // 动作指示：灯亮 = 已分离
}

// 执行开伞动作：CH3 舵机释放主伞
static void DoParachute(void)
{
	Servo_SetUMBAngle(SERVO_UMB_RELEASE_ANGLE);
}

/* ========================================== 对外接口 ========================================== */

void Separation_Init(void)
{
	// 先初始化 PWM/舵机外设，保证后续能立刻写入角度（内部自带重复调用保护：重复调用 PWM_Init 无害）
	Servo_Init();

	// 让两个舵机先处于"锁定"位置
	Servo_SetPlaneAngle(SERVO_PLANE_LOCK_ANGLE);
	Servo_SetUMBAngle(SERVO_UMB_LOCK_ANGLE);

	LED1_OFF();
	State         = SEP_STATE_IDLE;
	TmrCnt        = 0;
	WeightlessCnt = 0;
}

// 由调度器每 10ms 调用一次（Task_Flight）
// 采样已由 Task_IMU（AccFilter_Update）负责，本函数只读滤波结果做状态判定
void Separation_Loop(void)
{
	float mag;

	switch(State)
	{
	/* -------------------- 状态 0：待发，等待起飞确认 -------------------- */
	case SEP_STATE_IDLE:
	{
		// 此时合加速度在 1.0g 附近是正常的
		mag = AccFilter_GetMagnitude();

		if(mag > LAUNCH_THRESHOLD_G)
		{
			TmrCnt ++;
			if(TmrCnt >= LAUNCH_CONFIRM_CNT)
			{
				// 连续 100ms 受到大推力 → 确认已经起飞
				State  = SEP_STATE_ASCENT;
				TmrCnt = 0;
			}
		}
		else
		{
			// 中间一旦回落就清零——只有连续满足才算
			TmrCnt = 0;
		}
	}
	break;

	/* -------------------- 状态 1：动力段，等加速度回落 -------------------- */
	case SEP_STATE_ASCENT:
	{
		mag = AccFilter_GetMagnitude();

		// 加速度持续降到滑行水平 → 确认动力耗尽
		if(mag < COAST_THRESHOLD_G)
		{
			TmrCnt ++;
			if(TmrCnt >= COAST_CONFIRM_CNT)
			{
				State  = SEP_STATE_COAST;
				TmrCnt = 0;          // 复用作滑行计时 + 超时计数
			}
		}
		else
		{
			TmrCnt = 0;
		}
	}
	break;

	/* -------------------- 状态 2：滑行中，等待失重触发 -------------------- */
	case SEP_STATE_COAST:
	{
		uint8_t hit_weightless, hit_timeout;

		mag = AccFilter_GetMagnitude();

		// 条件一：失重持续满足
		if(mag < WEIGHTLESS_THRESHOLD_G)
		{
			WeightlessCnt ++;
		}
		else
		{
			WeightlessCnt = 0;
		}

		// 条件二：超时保险（滑行太久仍强制分离，避免高空不开伞）
		hit_weightless = (WeightlessCnt >= WEIGHTLESS_CONFIRM_CNT);
		hit_timeout    = (TmrCnt     >= COAST_TIMEOUT_CNT);

		if(hit_weightless || hit_timeout)
		{
			DoSeparation();                      // 执行 CH2 舵机分离
			WeightlessCnt = 0;
			State  = SEP_STATE_SEP_DONE;
			TmrCnt = 0;                          // 复用为开伞延时计数
		}
		else
		{
			TmrCnt ++;
		}
	}
	break;

	/* -------------------- 状态 3：已分离，等延时开伞 -------------------- */
	case SEP_STATE_SEP_DONE:
	{
		TmrCnt ++;
		if(TmrCnt >= PARACHUTE_DELAY_CNT)
		{
			DoParachute();                        // 执行 CH3 舵机开伞
			State = SEP_STATE_DONE;
			TmrCnt = 0;
		}
	}
	break;

	/* -------------------- 状态 4：完成 -------------------- */
	case SEP_STATE_DONE:
	default:
	{
		// 整个流程结束，保持空闲即可
	}
	break;
	}
}

SepState_t Separation_GetState(void)
{
	return State;
}
