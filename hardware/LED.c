#include "stm32f10x.h"                  // Device header

/* 引脚说明：
 * 原引脚 PA1/PA2 与 TIM2 舵机 PWM（CH2/CH3）冲突，已挪至安全引脚。
 * LED1 = PC13：最小系统板载LED，低电平点亮，无需额外接线
 * LED2 = PB5  ：如接外部LED模块：PB5 -> 限流电阻 -> LED -> 3.3V
 * 所有 LED 函数保持低电平有效语义，接口名不变，调用方无需修改
 */
#define LED1_PORT   GPIOC
#define LED1_PIN    GPIO_Pin_13
#define LED2_PORT   GPIOB
#define LED2_PIN    GPIO_Pin_5

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = LED1_PIN;
	GPIO_Init(LED1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED2_PIN;
	GPIO_Init(LED2_PORT, &GPIO_InitStructure);

	GPIO_SetBits(LED1_PORT, LED1_PIN);
	GPIO_SetBits(LED2_PORT, LED2_PIN);
}

void LED1_ON(void)
{
	GPIO_ResetBits(LED1_PORT, LED1_PIN);
}

void LED1_OFF(void)
{
	GPIO_SetBits(LED1_PORT, LED1_PIN);
}

void LED1_Turn(void)
{
	if(GPIO_ReadOutputDataBit(LED1_PORT, LED1_PIN) == 0)
	{
		GPIO_SetBits(LED1_PORT, LED1_PIN);
	}
	else
	{
		GPIO_ResetBits(LED1_PORT, LED1_PIN);
	}
}

void LED2_ON(void)
{
	GPIO_ResetBits(LED2_PORT, LED2_PIN);
}

void LED2_OFF(void)
{
	GPIO_SetBits(LED2_PORT, LED2_PIN);
}

void LED2_Turn(void)
{
	if(GPIO_ReadOutputDataBit(LED2_PORT, LED2_PIN) == 0)
	{
		GPIO_SetBits(LED2_PORT, LED2_PIN);
	}
	else
	{
		GPIO_ResetBits(LED2_PORT, LED2_PIN);
	}
}
