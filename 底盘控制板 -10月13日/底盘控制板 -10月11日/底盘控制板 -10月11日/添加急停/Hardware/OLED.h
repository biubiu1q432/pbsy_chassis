#include "stm32f10x.h"

#ifndef __OLED_091_II2_H
#define __OLED_091_II2_H
 
#define OLED_MODE 0
#define SIZE 8
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	  
 
//-----------------OLED IIC端口定义----------------  					   

#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_4)//SCL
#define OLED_SCLK_Set() GPIO_SetBits(GPIOB,GPIO_Pin_4)

#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_5)//SDA
#define OLED_SDIN_Set() GPIO_SetBits(GPIOB,GPIO_Pin_5)

#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

void OLED_ShowString(u8 x,u8 y,u8 *chr);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_On(void);
void OLED_ShowNum(u8 x,u8 y,int16_t no ,uint16_t length);
#endif
