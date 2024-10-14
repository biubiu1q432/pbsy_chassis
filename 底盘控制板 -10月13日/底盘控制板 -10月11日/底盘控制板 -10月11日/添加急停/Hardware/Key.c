#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "freertos.h"
#include "task.h"
void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//没有用
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

uint8_t Key_Get(void)//unsigned char
{
	uint8_t Key = 0;
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8) == 0)
	{
		uint16_t timer = 0;
		vTaskDelay(10);
		while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8) == 0)	
		{
			vTaskDelay(10);
			timer++;
		}
		if(timer < 50)
		{
			Key = 1;
		}
		else 
		{
			Key = 2;
		}
		
		
	}

	return Key;
}
