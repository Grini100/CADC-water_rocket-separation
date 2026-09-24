#include "stm32f10x.h"                  // Device header
#include "PWM.h"


void Servo_Init(void)
{
	PWM_Init();
}

//0度--RCC=500
//180--RCC=2500
void Servo_SetPlaneAngle(float Angle)//分离舵机
{
	PWM_SetCompare2(Angle / 180 * 2000 + 500);//+500是偏移值
}

void Servo_SetUMBAngle(float Angle)//开伞舵机
{
	PWM_SetCompare3(Angle / 180 * 2000 + 500);
}
