#include "stm32f10x.h"                  // Device header
#include "VL6180.h"

uint8_t SENCER_NUM = 1;


void VL6180_I2C_SCL_1(void) 
{
	if(SENCER_NUM == 1)
		GPIOB->BSRR |= VL6180_I2C_SCL_PIN1 ;		/* SCL = 1 */
	else if(SENCER_NUM == 2)
		GPIOB->BSRR |= VL6180_I2C_SCL_PIN2;
	else if(SENCER_NUM == 3)	
		GPIOC->BSRR |= VL6180_I2C_SCL_PIN3;
	else if(SENCER_NUM == 4)
		GPIOC->BSRR |= VL6180_I2C_SCL_PIN4;
}
void VL6180_I2C_SCL_0(void)  
{
	if(SENCER_NUM == 1)
		GPIOB->BRR  |= VL6180_I2C_SCL_PIN1;		/* SCL = 0 */
	else if(SENCER_NUM == 2)
		GPIOB->BRR  |= VL6180_I2C_SCL_PIN2;
	else if(SENCER_NUM == 3)
		GPIOC->BRR  |= VL6180_I2C_SCL_PIN3;
	else if(SENCER_NUM == 4)
		GPIOC->BRR  |= VL6180_I2C_SCL_PIN4;
}
void VL6180_I2C_SDA_1()
{
	if(SENCER_NUM == 1)
		GPIOB->BSRR |= VL6180_I2C_SDA_PIN1;		/* SDA = 1 */
	else if(SENCER_NUM == 2)
		GPIOB->BSRR |= VL6180_I2C_SDA_PIN2;
	else if(SENCER_NUM == 3)
		GPIOC->BSRR |= VL6180_I2C_SDA_PIN3;
	else if(SENCER_NUM == 4)
		GPIOA->BSRR |= VL6180_I2C_SDA_PIN4;
}
void VL6180_I2C_SDA_0()
{
	if(SENCER_NUM == 1)
		GPIOB->BRR  |= VL6180_I2C_SDA_PIN1;		/* SDA = 0 */
	else if(SENCER_NUM == 2)
		GPIOB->BRR  |= VL6180_I2C_SDA_PIN2;		
	else if(SENCER_NUM == 3)
		GPIOC->BRR  |= VL6180_I2C_SDA_PIN3;		
	else if(SENCER_NUM == 4)
		GPIOA->BRR  |= VL6180_I2C_SDA_PIN4;
}

uint8_t VL6180_I2C_SDA_READ()
{
	if(SENCER_NUM == 1)
		return ((GPIOB->IDR & VL6180_I2C_SDA_PIN1)!=0 );	/* 读SDA口线状态 */
	else if(SENCER_NUM == 2)
		return ((GPIOB->IDR & VL6180_I2C_SDA_PIN2)!=0 );	/* 读SDA口线状态 */
	else if(SENCER_NUM == 3)
		return ((GPIOC->IDR & VL6180_I2C_SDA_PIN3)!=0 );	/* 读SDA口线状态 */	
	else if(SENCER_NUM == 4)
		return ((GPIOA->IDR & VL6180_I2C_SDA_PIN4)!=0 );	/* 读SDA口线状态 */	
	else return 0;
}


static void VL6180_Delay(void)
{
	uint8_t i;
//	for(i=0;i<5;i++)
//	{
//		
//	}
}
void VL6180_Start(void)
{
	VL6180_I2C_SCL_1();
	VL6180_I2C_SDA_1();
	VL6180_Delay();
	VL6180_I2C_SDA_0();
	VL6180_Delay();
	VL6180_I2C_SCL_0();
	VL6180_Delay();
}
void VL6180_Stop(void)
{
	VL6180_I2C_SDA_0();
	VL6180_I2C_SCL_1();
	VL6180_Delay();
	VL6180_I2C_SDA_1();
	VL6180_Delay();
}
void VL6180_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);//使能GPIOA,GPIOB,GPIOC,AFIO;
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = VL6180_I2C_SDA_PIN4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = VL6180_I2C_SCL_PIN1|VL6180_I2C_SDA_PIN1|VL6180_I2C_SCL_PIN2|VL6180_I2C_SDA_PIN2;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = VL6180_I2C_SCL_PIN3|VL6180_I2C_SDA_PIN3|VL6180_I2C_SCL_PIN4;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA,VL6180_I2C_SDA_PIN4);
	GPIO_SetBits(GPIOB,VL6180_I2C_SCL_PIN1|VL6180_I2C_SDA_PIN1|VL6180_I2C_SCL_PIN2|VL6180_I2C_SDA_PIN2);
	GPIO_SetBits(GPIOC,VL6180_I2C_SCL_PIN3|VL6180_I2C_SDA_PIN3|VL6180_I2C_SCL_PIN4);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin =  VL6180_SHDN1|VL6180_SHDN2|VL6180_SHDN3;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  VL6180_SHDN4;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* 给一个停止信号, 复位I2C总线上的所有设备到待机模式 */
	SENCER_NUM = 1;VL6180_Stop();SENCER_NUM = 2;VL6180_Stop();SENCER_NUM = 3;VL6180_Stop();SENCER_NUM = 4;VL6180_Stop();
}

void VL6180_Ack(void)
{
	VL6180_I2C_SCL_0();
	VL6180_Delay();
	VL6180_I2C_SDA_0();
	VL6180_Delay();
	VL6180_I2C_SCL_1();
	VL6180_Delay();
	VL6180_I2C_SCL_0();
	VL6180_Delay();
	VL6180_I2C_SDA_1();
	VL6180_Delay();

}

void VL6180_NAcK(void)
{
	VL6180_I2C_SDA_1();
	VL6180_Delay();
	VL6180_I2C_SCL_1();
	VL6180_Delay();
	VL6180_I2C_SCL_0();
	VL6180_Delay();

}

uint8_t VL6180_WaitAck(void)
{
	uint8_t ret;
	VL6180_I2C_SDA_1();
	VL6180_I2C_SCL_1();
	VL6180_Delay();
	if( VL6180_I2C_SDA_READ() )
	{
		ret=1;
	}
	else
	{
		ret=0;
	}
	VL6180_I2C_SCL_0();
	VL6180_Delay();
  return ret;

}
	

void VL6180_SendByte(uint8_t data)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		if( data&0x80 )
	 {
		  VL6180_I2C_SDA_1();
	 }
	 else
	 {
		  VL6180_I2C_SDA_0();
	 }
	 VL6180_Delay();
	 VL6180_I2C_SCL_1();
	 VL6180_Delay();
	 VL6180_I2C_SCL_0();
	 VL6180_Delay();
	 if( i==7 )
	 {
		 VL6180_I2C_SDA_1();
		 VL6180_Delay();
	 }
	 data=data<<1;
	}
	
}

uint8_t VL6180_ReadByte(void)
{
	uint8_t value=0;
	uint8_t i;
	for(i=0;i<8;i++)
	{
		value=value<<1;
		VL6180_I2C_SCL_1();
	  VL6180_Delay();
		if( VL6180_I2C_SDA_READ() )
	  {
	 	  value++;
	  }
	  VL6180_I2C_SCL_0();
	  VL6180_Delay();
	}
	return value;
}

uint8_t VL6180_CheckDevice(uint8_t Address)
{
	uint8_t ucACK;
//	VL6180_GPIO_Config();
	VL6180_Start();
	VL6180_SendByte(Address|VL6180_I2C_WR);
	ucACK=VL6180_WaitAck();
	VL6180_Stop();
  return ucACK;	
	
}




//写	reg寄存器 data数据
u8 VL6180X_WriteByte(u16 reg,u8 data)
{
	uint8_t Index_H = (uint8_t)(reg >> 8);
	uint8_t Index_L = (uint8_t)(reg & 0xFF);
	
	VL6180_Start();
	VL6180_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|0);
	if(VL6180_WaitAck())	//等待应答
	{
		VL6180_Stop();	
		return 1;		
	}
	VL6180_SendByte(Index_H);
	VL6180_WaitAck();	//等待ACK
	VL6180_SendByte(Index_L);
	VL6180_WaitAck();	//等待ACK
	VL6180_SendByte(data);
	if(VL6180_WaitAck())	//等待ACK
	{
		VL6180_Stop();	 
		return 1;		 
	}
	VL6180_Stop();
	return 0;	
}

//VL6180X读取8位数据
u8 VL6180X_ReadByte(u16 reg)
{
	u8 res;
	uint8_t Index_H = (uint8_t)(reg >> 8);
	uint8_t Index_L = (uint8_t)(reg & 0xff);
    VL6180_Start(); 
	
	VL6180_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|0);//发送器件地址+写命令	

	VL6180_WaitAck();		//等待应答 
    VL6180_SendByte(Index_H);	//写寄存器地址
    VL6180_WaitAck();		//等待应答
	VL6180_SendByte(Index_L);	//写寄存器地址
	VL6180_WaitAck();	
	
    VL6180_Start();

	VL6180_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|1);//发送器件地址+读命令	
    VL6180_WaitAck();		//等待应答 
	res=VL6180_ReadByte();//读取数据,发送nACK 
	VL6180_NAcK();
	
    VL6180_Stop();			//产生一个停止条件 
	return res;
}
uint8_t VL6180X_Read_ID(void)
{
	return VL6180X_ReadByte(VL6180X_REG_IDENTIFICATION_MODEL_ID);
}
uint8_t VL6180X_Init(uint8_t NUM)
{
	SENCER_NUM = NUM;
	//printf("%d",VL6180X_Read_ID());
	if(VL6180X_Read_ID() == VL6180X_DEFAULT_ID)
	{	
		VL6180X_WriteByte(0x0207, 0x01);
		VL6180X_WriteByte(0x0208, 0x01);
		VL6180X_WriteByte(0x0096, 0x00);
		VL6180X_WriteByte(0x0097, 0xfd);
		VL6180X_WriteByte(0x00e3, 0x00);
		VL6180X_WriteByte(0x00e4, 0x04);
		VL6180X_WriteByte(0x00e5, 0x02);
		VL6180X_WriteByte(0x00e6, 0x01);
		VL6180X_WriteByte(0x00e7, 0x03);
		VL6180X_WriteByte(0x00f5, 0x02);
		VL6180X_WriteByte(0x00d9, 0x05);
		VL6180X_WriteByte(0x00db, 0xce);
		VL6180X_WriteByte(0x00dc, 0x03);
		VL6180X_WriteByte(0x00dd, 0xf8);
		VL6180X_WriteByte(0x009f, 0x00);
		VL6180X_WriteByte(0x00a3, 0x3c);
		VL6180X_WriteByte(0x00b7, 0x00);
		VL6180X_WriteByte(0x00bb, 0x3c);
		VL6180X_WriteByte(0x00b2, 0x09);
		VL6180X_WriteByte(0x00ca, 0x09);
		VL6180X_WriteByte(0x0198, 0x01);
		VL6180X_WriteByte(0x01b0, 0x17);
		VL6180X_WriteByte(0x01ad, 0x00);
		VL6180X_WriteByte(0x00ff, 0x05);
		VL6180X_WriteByte(0x0100, 0x05);
		VL6180X_WriteByte(0x0199, 0x05);
		VL6180X_WriteByte(0x01a6, 0x1b);
		VL6180X_WriteByte(0x01ac, 0x3e);
		VL6180X_WriteByte(0x01a7, 0x1f);
		VL6180X_WriteByte(0x0030, 0x00);
		
		// Recommended : Public registers - See data sheet for more detail
		VL6180X_WriteByte(0x0011, 0x10);       // Enables polling for 'New Sample ready'
									// when measurement completes
		VL6180X_WriteByte(0x010a, 0x30);       // Set the averaging sample period
									// (compromise between lower noise and
									// increased execution time)
		VL6180X_WriteByte(0x003f, 0x46);       // Sets the light and dark gain (upper
									// nibble). Dark gain should not be
									// changed. !上半字节要写入0x4	默认增益是1.0
		VL6180X_WriteByte(0x0031, 0xFF);       // sets the # of range measurements after
									// which auto calibration of system is
									// performed
		VL6180X_WriteByte(0x0040, 0x63);       // Set ALS integration time to 100ms
		VL6180X_WriteByte(0x002e, 0x01);       // perform a single temperature calibration
									// of the ranging sensor

		// Optional: Public registers - See data sheet for more detail
		VL6180X_WriteByte(0x001b, 0x09);    //测量间隔	轮询模式
									// period to 100ms	每步10ms->0-10ms
		VL6180X_WriteByte(0x003e, 0x31);      //测量周期	ALS模式
									// to 500ms		
		VL6180X_WriteByte(0x0014, 0x24);       // Configures interrupt on 'New Sample
									// Ready threshold event'
									
				
		if(SENCER_NUM == 1)
		{
			VL6180X_WriteByte(0x024,100);//RF
		}
		else if(SENCER_NUM == 2)
		{
			VL6180X_WriteByte(0x024,40);//RB
		}
		else if(SENCER_NUM == 3)
		{
			VL6180X_WriteByte(0x024,25);//LF
		}
		else if(SENCER_NUM == 4)
		{
			VL6180X_WriteByte(0x024,85);//LB  110
		}		
				
									
		return 0;
	}

	else 	printf("2\r\n");
	return 1;	

}
//单位毫米
uint8_t VL6180X_Read_Range(void)
{
	uint8_t range = 0;
	//开启传输
	while(!(VL6180X_ReadByte(VL6180X_REG_RESULT_RANGE_STATUS) & 0x01));
	VL6180X_WriteByte(VL6180X_REG_SYSRANGE_START,0x01);	//单次触发模式
	
	//等待新样本就绪阈值事件(New Sample Ready threshold event)
	while(!(VL6180X_ReadByte(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04));
	
	range = VL6180X_ReadByte(VL6180X_REG_RESULT_RANGE_VAL);
	//获取完数据，清楚中断位
	VL6180X_WriteByte(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR,0x07);	//0111b 清除了三种中断标志
	return range;
}
//VL6180X读取16位数据
//uint8_t VL6180X_Read_HalfWold(u16 reg)
//{
//	u16 res;
//	uint8_t Index_H = (uint8_t)(reg >> 8);
//	uint8_t Index_L = (uint8_t)(reg & 0xff);
//    VL6180_Start(); 
//	VL6180_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|0);//发送器件地址+写命令	
//	VL6180_WaitAck();		//等待应答 
//    VL6180_SendByte(Index_H);	//写寄存器地址
//    VL6180_WaitAck();		//等待应答
//	VL6180_SendByte(Index_L);	//写寄存器地址
//	VL6180_WaitAck();	
//	
//    VL6180_Start();
//	VL6180_SendByte((VL6180X_DEFAULT_I2C_ADDR<<1)|1);//发送器件地址+读命令	
//    VL6180_WaitAck();		//等待应答 
//	res = VL6180_ReadByte();//读取数据,发送ACK 
//	VL6180_Ack();
//	res <<= 8;
//	res |= VL6180_ReadByte();//读取数据,发送nACK 
//	VL6180_NAcK();
//    VL6180_Stop();			//产生一个停止条件 
//	return res;
//}

//float VL6180X_Read_Lux(uint8_t Gain)
//{
//	float lux;
//	uint8_t reg;
//	reg = VL6180X_ReadByte(VL6180X_REG_SYSTEM_INTERRUPT_CONFIG);
//	reg &= ~0x38;		//[5:3]清0
//	reg |= (0x4<<3);	//开启转换New sample ready	开启转换
//	
//	VL6180X_WriteByte(VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI,0);
//	VL6180X_WriteByte(VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO,100);	//101ms
//	if (Gain > VL6180X_ALS_GAIN_40)
//	{
//		Gain = VL6180X_ALS_GAIN_40;
//	}
//	VL6180X_WriteByte(VL6180X_REG_SYSALS_ANALOGUE_GAIN, 0x40 | Gain);
//	VL6180X_WriteByte(VL6180X_REG_SYSALS_START, 0x1);	//连续模式
//	// New Sample Ready threshold event 新样本就绪
//	while (4 != ((VL6180X_ReadByte(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) >> 3) & 0x7));
//	
//	lux = VL6180X_Read_HalfWold(VL6180X_REG_RESULT_ALS_VAL);
//	VL6180X_WriteByte(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR,0x07);	//0111b 清除了三种中断标志
//	//矫正增益算法
//	lux *= 0.32f; // calibrated count/lux
//	switch(Gain) { 
//	case VL6180X_ALS_GAIN_1: 
//	break;
//	case VL6180X_ALS_GAIN_1_25: 
//	lux /= 1.25f;
//	break;
//	case VL6180X_ALS_GAIN_1_67: 
//	lux /= 1.76f;
//	break;
//	case VL6180X_ALS_GAIN_2_5: 
//	lux /= 2.5f;
//	break;
//	case VL6180X_ALS_GAIN_5: 
//	lux /= 5;
//	break;
//	case VL6180X_ALS_GAIN_10: 
//	lux /= 10;
//	break;
//	case VL6180X_ALS_GAIN_20: 
//	lux /= 20;
//	break;
//	case VL6180X_ALS_GAIN_40: 
//	lux /= 20;
//	break;
//	}
//	lux *= 100;
//	lux /= 100; // integration time in ms
//	return lux;
//}

//int  VL6180x_SetOffsetCalibrationData(VL6180xDev_t dev, int8_t offset)
//{
//	int status;
////	LOG_FUNCTION_START("%d", offset);
//	
//	
//	
//	VL6180xDevDataSet(dev, Part2PartOffsetNVM, offset);
//	
//	
//	
//	offset /= _GetUpscale(dev);
//	status = VL6180x_WrByte(dev, SYSRANGE_PART_TO_PART_RANGE_OFFSET, offset);
//	LOG_FUNCTION_END(status);
//	return status;
//}





uint8_t Side_Range(uint8_t Num)
{
	SENCER_NUM = Num;
	return VL6180X_Read_Range();
	
}
