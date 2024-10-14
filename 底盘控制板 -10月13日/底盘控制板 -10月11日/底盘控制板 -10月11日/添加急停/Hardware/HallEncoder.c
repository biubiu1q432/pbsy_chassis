#include "stm32f10x.h"                  // Device header
#include "HallEncoder.h"  

void Hal01_Init(void)//»ô¶û±àÂëÆ÷1³õÊ¼»¯
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;//TIM8
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;			//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1-1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_InitStructure;
	TIM_ICStructInit(&TIM_InitStructure);
//	TIM_InitStructure.TIM_Channel = TIM_Channel_1;
	TIM_InitStructure.TIM_ICFilter = 0x0A;
	TIM_ICInit(TIM8, &TIM_InitStructure);
	
	TIM_EncoderInterfaceConfig(TIM8,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	
	TIM_Cmd(TIM8,ENABLE);
}

void Hal02_Init(void)//»ô¶û±àÂëÆ÷2³õÊ¼»¯
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;//TIM1
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536-1;			//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1-1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitTypeDef TIM_InitStructure;
	TIM_ICStructInit(&TIM_InitStructure);
//	TIM_InitStructure.TIM_Channel = TIM_Channel_1;
	TIM_InitStructure.TIM_ICFilter = 0x0A;
	TIM_ICInit(TIM5, &TIM_InitStructure);
	
	TIM_EncoderInterfaceConfig(TIM5,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	
	TIM_Cmd(TIM5,ENABLE);
}


int16_t Conter1_Get(void)
{
	int16_t Temp;
	Temp = TIM_GetCounter(TIM8);
	TIM_SetCounter(TIM8,0);
	return Temp;
}
int16_t Conter2_Get(void)
{
	int16_t Temp;
	Temp = TIM_GetCounter(TIM5);
	TIM_SetCounter(TIM5,0);
	return Temp;
}
//int16_t Conter3_Get(void)
//{
//	int16_t Temp;
//	Temp = TIM_GetCounter(TIM8);
//	TIM_SetCounter(TIM2,0);
//	return Temp;
//}
//int16_t Conter4_Get(void)
//{
//	int16_t Temp;
//	Temp = TIM_GetCounter(TIM8);
//	TIM_SetCounter(TIM5,0);
//	return Temp;
//}






