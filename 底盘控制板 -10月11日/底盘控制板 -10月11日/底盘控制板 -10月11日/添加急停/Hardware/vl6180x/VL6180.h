#ifndef __VL6180_H
#define __VL6180_H
#include "stm32f10x.h"

#define VL6180_I2C_WR	0		/* Ð´¿ØÖÆbit */
#define VL6180_I2C_RD	1		/* ¶Á¿ØÖÆbit */

#define VL6180_GPIO_PORT_I2C         GPIOB
#define VL6180_RCC_I2C_PORT          RCC_APB2Periph_GPIOB

#define VL6180_GPIO_PORT_I2C1        GPIOC
#define VL6180_RCC_I2C_PORT1         RCC_APB2Periph_GPIOC

#define VL6180_GPIO_PORT_I2C2        GPIOA
#define VL6180_RCC_I2C_PORT2         RCC_APB2Periph_GPIOA

#define VL6180_I2C_SCL_PIN1          									GPIO_Pin_7
#define VL6180_I2C_SDA_PIN1          									GPIO_Pin_8
#define VL6180_SHDN1                									GPIO_Pin_6

#define VL6180_I2C_SCL_PIN2          									GPIO_Pin_4
#define VL6180_I2C_SDA_PIN2          									GPIO_Pin_5
#define VL6180_SHDN2                 									GPIO_Pin_3

#define VL6180_I2C_SCL_PIN3          									GPIO_Pin_6
#define VL6180_I2C_SDA_PIN3          									GPIO_Pin_7
#define VL6180_SHDN3                 									GPIO_Pin_15

#define VL6180_I2C_SCL_PIN4          									GPIO_Pin_9
#define VL6180_I2C_SDA_PIN4         								 	GPIO_Pin_8
#define VL6180_SHDN4                 								 	GPIO_Pin_8



#define	VL6180X_DEFAULT_ID						0xB4
//#define I2C_DEBUG
#define VL6180X_DEFAULT_I2C_ADDR 				0x29  ///< The fixed I2C addres
/*------------------VL6180XÄÚ²¿¼Ä´æÆ÷------------------*/
///! Device model identification number
#define VL6180X_REG_IDENTIFICATION_MODEL_ID    0x000
///! Interrupt configuration
#define VL6180X_REG_SYSTEM_INTERRUPT_CONFIG    0x014
///! Interrupt clear bits
#define VL6180X_REG_SYSTEM_INTERRUPT_CLEAR     0x015
///! Fresh out of reset bit
#define VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET  0x016
///! Trigger Ranging
#define VL6180X_REG_SYSRANGE_START             0x018
///! Trigger Lux Reading
#define VL6180X_REG_SYSALS_START               0x038
///! Lux reading gain
#define VL6180X_REG_SYSALS_ANALOGUE_GAIN       0x03F
///! Integration period for ALS mode, high byte
#define VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI  0x040
///! Integration period for ALS mode, low byte
#define VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO  0x041
///! Specific error codes
#define VL6180X_REG_RESULT_RANGE_STATUS        0x04d
///! Interrupt status
#define VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO       0x04f
///! Light reading value
#define VL6180X_REG_RESULT_ALS_VAL             0x050
///! Ranging reading value
#define VL6180X_REG_RESULT_RANGE_VAL           0x062

#define VL6180X_ALS_GAIN_1         0x06  ///< 1x gain
#define VL6180X_ALS_GAIN_1_25      0x05  ///< 1.25x gain
#define VL6180X_ALS_GAIN_1_67      0x04  ///< 1.67x gain
#define VL6180X_ALS_GAIN_2_5       0x03  ///< 2.5x gain
#define VL6180X_ALS_GAIN_5         0x02  ///< 5x gain
#define VL6180X_ALS_GAIN_10        0x01  ///< 10x gain
#define VL6180X_ALS_GAIN_20        0x00  ///< 20x gain
#define VL6180X_ALS_GAIN_40        0x07  ///< 40x gain

#define VL6180X_ERROR_NONE         0   ///< Success!
#define VL6180X_ERROR_SYSERR_1     1   ///< System error
#define VL6180X_ERROR_SYSERR_5     5   ///< Sysem error
#define VL6180X_ERROR_ECEFAIL      6   ///< Early convergence estimate fail
#define VL6180X_ERROR_NOCONVERGE   7   ///< No target detected
#define VL6180X_ERROR_RANGEIGNORE  8   ///< Ignore threshold check failed
#define VL6180X_ERROR_SNR          11  ///< Ambient conditions too high
#define VL6180X_ERROR_RAWUFLOW     12  ///< Raw range algo underflow
#define VL6180X_ERROR_RAWOFLOW     13  ///< Raw range algo overflow
#define VL6180X_ERROR_RANGEUFLOW   14  ///< Raw range algo underflow
#define VL6180X_ERROR_RANGEOFLOW   15  ///< Raw range algo overflow

void VL6180_GPIO_Config(void);
uint8_t VL6180X_Init(uint8_t NUM);
uint8_t VL6180_CheckDevice(uint8_t Address);
uint8_t Side_Range(uint8_t Num);

#endif
