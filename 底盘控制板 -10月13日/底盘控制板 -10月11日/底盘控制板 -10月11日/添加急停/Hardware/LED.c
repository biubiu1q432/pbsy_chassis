#include "stm32f10x.h"                  // Device header
#include "Delay.h"

void LED_ON(void)
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_13 );
}
void LED_OFF(void)
{
	GPIO_SetBits(GPIOC,GPIO_Pin_13 );
}

void BEEP_OFF(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_1 );
}
void BEEP_ON(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_1 );
}

void LED_Turn(void)
{
	if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13 )==0)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_13 );
	}
	else
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_13 );
	}
}



void LED_Init(void)
{
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);//LED
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_1;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);//BEEP
	
	BEEP_ON();
	delay_ms(5);
	BEEP_OFF();
	delay_ms(95);
	BEEP_ON();
	delay_ms(5);
	BEEP_OFF();
	delay_ms(95);			
	BEEP_ON();
	delay_ms(5);
	BEEP_OFF();
	delay_ms(95);	
}

void BEEP(uint8_t times)
{
	uint8_t i;
	for(i=0;i<times;i++)
    {
		BEEP_ON();
		vTaskDelay(5);
		BEEP_OFF();
		vTaskDelay(95);		
    }
}
