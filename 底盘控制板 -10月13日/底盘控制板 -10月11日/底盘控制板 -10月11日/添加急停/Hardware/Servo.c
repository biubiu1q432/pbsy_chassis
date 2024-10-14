#include "stm32f10x.h"                  // Device header

void PWM_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare1(TIM4,Compare);
}
void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare2(TIM4,Compare);
}


void Servo_SetAngle(float Angle)
{
	PWM_SetCompare1(Angle/180*2000+500);
	
}

void Light(float light_level)
{
	if(light_level == 0)
	{
		PWM_SetCompare2(0);
	}
	else if(light_level == 1)
	{
		PWM_SetCompare2(5000);
	}
	else if(light_level == 2)
	{
		PWM_SetCompare2(10000);
	}
	else if(light_level == 3)
	{
		PWM_SetCompare2(15000);
	}
	else if(light_level == 4)
	{
		PWM_SetCompare2(20000);
	}
	else
	{
		PWM_SetCompare2(0);
	}
}
void Servo_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM4);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 20000-1;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 72-1;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	//TIM_OCInitStructure.TIM_OCIdleState =;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	
	TIM_Cmd(TIM4, ENABLE);
	
	Servo_SetAngle(30);
	PWM_SetCompare2(0);
}