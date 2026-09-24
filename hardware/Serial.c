#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "OLED.H"

uint8_t Serial_RxFlag;
uint8_t Serial_TxPacket[4];
uint8_t Serial_RxPacket[4];


void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//����USART1ʱ�ӣ�ֻ��USART1��APB2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//�������죬���ŵ�ƽ���������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;//USART1��TX(��������)��PA9��
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	// 修复：RX必须上拉IPU。UART空闲电平为高，若用下拉IPD，
	// USB-TTL未接/断开时RX悬空为低=恒break帧，会以约960Hz触发RXNE中断
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;

	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=115200;//������ֱ��д����
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//������Ŀǰ����Ҫ��
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;//���͹���
	USART_InitStructure.USART_Parity=USART_Parity_No;//��У��
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλռ1
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//�ֳ�Ϊ8bit
	
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;//����ͨ��
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE);//����USART1
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);//�����ֽں���
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);//�鿴��־λ���ж������Ƿ�ӷ��ͼĴ������䵽��λ�Ĵ�������ֹ���ݸ���
	//�ñ�־λ�����´ε��÷����ֽں���ʱ���Զ����
}

void Serial_SendArray(uint8_t *Array,uint16_t Length)//发送数组
{
	uint16_t i;                     // 修复：uint8_t在Length>255时回绕（黑匣子按256B页发送会踩中）
	for(i=0;i<Length;i++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)//�����ַ���
{
	for(uint8_t i=0;String[i]!=0;i++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint8_t x,uint8_t y)//次方函数
{
	// 修复：初值必须为1（原为0会导致任何次方都返回0，
	// Serial_SendNumber里将发生除零→HardFault）；类型升为uint32_t防Length>=3时溢出
	uint32_t Result=1;
	while(y--)
	{
		Result *=x;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Num,uint8_t Length)//�������֣��߼�Ϊ��ĳһλ�������г��Ը�λ����Ȼ����ж�10���࣬����12345����12345/10000��%10=1
{
	for(uint8_t i=0;i<Length;i++)
	{
	Serial_SendByte((Num/Serial_Pow(10,Length-i-1))%10+'0');//��0������Ϊ���������������Ǵ�����������ַ�������ƫ�Ʋ��ܱ�ʾ���ַ������Բο�ASCill���
	}
}

int fputc(int ch,FILE *f)//printf�����Ƕ�fputc���з�װ��ͨ��fputc����һ���ֽ�һ���ֽڵķ��ͣ��޸�fputc�����з��͵�Ŀ�ĵؽ����޸ľͿ��Է��͵����ڶ�������Ļ
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Print(char *format,...)//�ɱ�����Ĳ�������Ҫ��ѧ
{
	char String[100];
	va_list arg;
	va_start(arg,format);
	vsprintf(String,format,arg);
	va_end(arg);
	Serial_SendString(String);
}

void Serial_SendPacket(void)
{
	Serial_SendByte(0xFF);
	Serial_SendArray(Serial_TxPacket,4);
	Serial_SendByte(0xFE);
}

uint8_t Serial_GetRxFlag(void)
{
	if(Serial_RxFlag==1)
	{
		Serial_RxFlag=0;
		return 1;
	}
	return 0;
}

void USART1_IRQHandler(void)
{
	static uint8_t RxState=0;
	static uint8_t pRxPacket=0;

	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
	{
		uint8_t RxData = USART_ReceiveData(USART1);
		
		if(RxState==0)//������߼������Լ���һ�£����ض�˵
		{
			if(RxData==0xFF)
			{
				RxState=1;
				pRxPacket=0;
			}
		}
		else if(RxState==1)
		{
			Serial_RxPacket[pRxPacket]=RxData;
			pRxPacket++;
			if(pRxPacket>=4)
			{
				RxState=2;
			}
		}
		else if(RxState==2)
		{
			if(RxData==0xFE)
			{
				RxState=0;
				Serial_RxFlag=1;
			}
		}
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}
/*    �������ݰ����߼����յ�һ�����ݾͶ�ȡһ������
			���ܻ���������������º������ݸ�����ǰ������
*/
