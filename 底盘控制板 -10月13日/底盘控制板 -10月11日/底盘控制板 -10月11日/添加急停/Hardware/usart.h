#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define UART1_ON				1
#define UART2_ON				1
#define UART3_ON				1
#define UART4_ON				1

#define UART1_BUS				RCC_APB2Periph_USART1
#define UART1_APB				RCC_APB2Periph_GPIOA
#define UART1_PORT				GPIOA
#define UART1_RX_PIN			GPIO_Pin_10
#define UART1_TX_PIN			GPIO_Pin_9
#define UART1_BaudRate			115200
#define UART1_NAME				USART1

#define UART2_BUS				RCC_APB1Periph_USART2
#define UART2_APB				RCC_APB2Periph_GPIOA
#define UART2_PORT				GPIOA
#define UART2_RX_PIN			GPIO_Pin_3
#define UART2_TX_PIN			GPIO_Pin_2
#define UART2_BaudRate			115200
#define UART2_NAME				USART2



void Serial_Init(void);
#endif


