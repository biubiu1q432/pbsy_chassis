#include "sys.h"
#include "tcs34725.h"
#include "Delay.h"
/******************************************************************************/

unsigned short  lux;
/******************************************************************************/
void TCS34725_I2C_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;//PB10/PB10=���I2C
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//ͨ���������	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//�ٶ�
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ѡ�йܽų�ʼ��
	GPIO_SetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_13);   //�ߵ�ƽ
}
/*********************************************/

void TCS_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable the clock for GPIO port D
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;   // Input mode with pull-down
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}


void TCS_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable the clock for GPIO port D
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   // Output mode, push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void TCS34725_I2C_Start()
{
		
	TCS_SDA_OUT();

	TCS_SDA_H;
	TCS_SCL_H;

	delay_us_iic(4);//delay_us_iic(4);
	TCS_SDA_L;	

	delay_us_iic(4);//delay_us_iic(4);
	TCS_SCL_L;

}
/*********************************************/
void TCS34725_I2C_Stop()
{
	TCS_SDA_OUT();
	TCS_SCL_L;
	TCS_SDA_L;
	delay_us_iic(4);
	TCS_SCL_H;
	TCS_SDA_H;
	delay_us_iic(4);							   	
}
/*********************************************/
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 TCS34725_I2C_Wait_ACK()
{
	u32 t=0;

	TCS_SDA_IN();//SDA����Ϊ����

	TCS_SDA_H; 
	delay_us_iic(1);
	TCS_SCL_H; 
	delay_us_iic(1); 	

	while(TCS_SDA_READ)
	{	
		t++;
		if(t > 5)//250
		{
			TCS34725_I2C_Stop();
			return 1;
		}
	}
	TCS_SCL_L;
	return 0;	
}
/*********************************************/
//����ACKӦ��
void TCS34725_I2C_ACK()
{
	TCS_SCL_L;
	TCS_SDA_OUT();//sda�����
	TCS_SDA_L;
	delay_us_iic(2);
	TCS_SCL_H;
	delay_us_iic(2);
	TCS_SCL_L;
}
/*********************************************/
//������ACKӦ��		    
void TCS34725_I2C_NACK()
{
	TCS_SCL_L;
	TCS_SDA_OUT();//sda�����
	TCS_SDA_H;
	delay_us_iic(2);
	TCS_SCL_H;
	delay_us_iic(2);
	TCS_SCL_L;
}
/*********************************************/
//I2C����һ���ֽ�		  
void TCS34725_I2C_Send_Byte(u8 byte)
{
	u8 i;
	
	TCS_SDA_OUT();//sda�����
	TCS_SCL_L;//����ʱ�ӿ�ʼ���ݴ���
	for(i = 0; i < 8; i++)
	{
		if(((byte&0x80)>>7)==1)TCS_SDA_H;
		else
			TCS_SDA_L;
		byte <<= 1;
		
		delay_us_iic(2);
		TCS_SCL_H;
		delay_us_iic(2);
		TCS_SCL_L;
		delay_us_iic(2);
	} 
}
/*********************************************/
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 TCS34725_I2C_Read_Byte(u8 ack)
{
	u8 i,receive = 0;
	
	TCS_SDA_IN();
	for(i = 0; i < 8; i++)
	{
		TCS_SCL_L;
		delay_us_iic(2);
		TCS_SCL_H;
		receive <<= 1;
		if(TCS_SDA_READ) receive++;
		delay_us_iic(1);
	}
	if (!ack) TCS34725_I2C_NACK();//����nACK
	else TCS34725_I2C_ACK(); //����ACK 
	
	return receive;
}
/*********************************************/
/*******************************************************************************
 * @brief Writes data to a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer - Pointer to a buffer storing the transmission data.
 * @param bytesNumber - Number of bytes to write.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
*******************************************************************************/
void TCS34725_I2C_Write(u8 slaveAddress, u8* dataBuffer,u8 bytesNumber, u8 stopBit)
{
	u8 i = 0;

	TCS34725_I2C_Start();
	TCS34725_I2C_Send_Byte((slaveAddress << 1) | 0x00);	   //���ʹӻ���ַд����	
	TCS34725_I2C_Wait_ACK();
	
	for(i = 0; i < bytesNumber; i++)
	{
		TCS34725_I2C_Send_Byte(*(dataBuffer + i));
		TCS34725_I2C_Wait_ACK();
	}
	if(stopBit == 1) TCS34725_I2C_Stop();
}
/*******************************************************************************
 * @brief Reads data from a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer - Pointer to a buffer that will store the received data.
 * @param bytesNumber - Number of bytes to read.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
*******************************************************************************/
void TCS34725_I2C_Read(u8 slaveAddress, u8* dataBuffer, u8 bytesNumber, u8 stopBit)
{
	u8 i = 0;
	TCS34725_I2C_Start();

	TCS34725_I2C_Send_Byte((slaveAddress << 1) | 0x01);	   //���ʹӻ���ַ������
	TCS34725_I2C_Wait_ACK();
	for(i = 0; i < bytesNumber; i++)
	{	
		if(i == bytesNumber - 1)
		{
			*(dataBuffer + i) = TCS34725_I2C_Read_Byte(0);//��ȡ�����һ���ֽڷ���NACK
		}
		else
		{
			*(dataBuffer + i) = TCS34725_I2C_Read_Byte(1);
		}
	}
	if(stopBit == 1) TCS34725_I2C_Stop();

}
/*******************************************************************************
 * @brief Writes data into TCS34725 registers, starting from the selected
 *        register address pointer.
 *
 * @param subAddr - The selected register address pointer.
 * @param dataBuffer - Pointer to a buffer storing the transmission data.
 * @param bytesNumber - Number of bytes that will be sent.
 *
 * @return None.
*******************************************************************************/
void TCS34725_Write(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
    u8 sendBuffer[10] = {0, };
    u8 byte = 0;
    sendBuffer[0] = subAddr | TCS34725_COMMAND_BIT;
    for(byte = 1; byte <= bytesNumber; byte++)
    {
        sendBuffer[byte] = dataBuffer[byte - 1];
    }
	TCS34725_I2C_Write(TCS34725_ADDRESS, sendBuffer, bytesNumber + 1, 1);
}
/*******************************************************************************
 * @brief Reads data from TCS34725 registers, starting from the selected
 *        register address pointer.
 *
 * @param subAddr - The selected register address pointer.
 * @param dataBuffer - Pointer to a buffer that will store the received data.
 * @param bytesNumber - Number of bytes that will be read.
 *
 * @return None.
*******************************************************************************/
void TCS34725_Read(u8 subAddr, u8* dataBuffer, u8 bytesNumber)
{
	subAddr |= TCS34725_COMMAND_BIT;
	TCS34725_I2C_Write(TCS34725_ADDRESS, (u8*)&subAddr, 1, 0);
	TCS34725_I2C_Read(TCS34725_ADDRESS, dataBuffer, bytesNumber, 1);
}
/*******************************************************************************
 * @brief TCS34725���û���ʱ��
 *
 * @return None
*******************************************************************************/
void TCS34725_SetIntegrationTime(u8 time)
{
	TCS34725_Write(TCS34725_ATIME, &time, 1);
}
/*******************************************************************************
 * @brief TCS34725��������
 *
 * @return None
*******************************************************************************/
void TCS34725_SetGain(u8 gain)
{
	TCS34725_Write(TCS34725_CONTROL, &gain, 1);
}
/*******************************************************************************
 * @brief TCS34725ʹ��
 *
 * @return None
*******************************************************************************/
void TCS34725_Enable(void)
{
	u8 cmd = TCS34725_ENABLE_PON;
	
	TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
	cmd = TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN;
	TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
	//delay_s(600000);//delay_ms(3);//��ʱӦ�÷�������AEN֮��
}
/*******************************************************************************
 * @brief TCS34725ʧ��
 *
 * @return None
*******************************************************************************/
void TCS34725_Disable(void)
{
	u8 cmd = 0;
	
	TCS34725_Read(TCS34725_ENABLE, &cmd, 1);
	cmd = cmd & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
	TCS34725_Write(TCS34725_ENABLE, &cmd, 1);
}
/*******************************************************************************
 * @brief TCS34725��ʼ��
 *
 * @return ID - ID�Ĵ����е�ֵ
*******************************************************************************/
u8 TCS34725_Init(void)
{
	u8 id=0;
	TCS34725_I2C_Init(); 

	TCS34725_Read(TCS34725_ID, &id, 1);  //TCS34725 �� ID �� 0x44 ���Ը���������ж��Ƿ�ɹ�����,0x4D��TCS34727;	
	if(id==0x4D | id==0x44)
		{	
		
			TCS34725_SetIntegrationTime(TCS34725_INTEGRATIONTIME_24MS);
			TCS34725_SetGain(TCS34725_GAIN_1X);
			
			//TCS34725_Write(TCS34725_CONFIG_WLONG, &(TCS34725_WTIME_2_4MS), 1);
			
			TCS34725_Enable();
			return 1;
		}
	return 0;

}
/*******************************************************************************
 * @brief TCS34725��ȡ����ͨ������
 *
 * @return data - ��ͨ����ת��ֵ
*******************************************************************************/
u16 TCS34725_GetChannelData(u8 reg)
{
	u8 tmp[2] = {0,0};
	u16 data;
	
	TCS34725_Read(reg, tmp, 2);
	data = (tmp[1] << 8) | tmp[0];
	
	return data;
}
/*******************************************************************************
 * @brief TCS34725��ȡ����ͨ������
 *
 * @return 1 - ת����ɣ����ݿ���
 *   	   0 - ת��δ��ɣ����ݲ�����
*******************************************************************************/
//u8 TCS34725_GetRawData(COLOR_RGBC *rgbc)
//{
//	u8 status = TCS34725_STATUS_AVALID;
//	
//	TCS34725_Read(TCS34725_STATUS, &status, 1);
//	
//	if(status & TCS34725_STATUS_AVALID)
//	{
//		rgbc->c = TCS34725_GetChannelData(TCS34725_CDATAL);	
//		rgbc->r = TCS34725_GetChannelData(TCS34725_RDATAL);	
//		rgbc->g = TCS34725_GetChannelData(TCS34725_GDATAL);	
//		rgbc->b = TCS34725_GetChannelData(TCS34725_BDATAL);
//		return 1;
//	}
//	return 0;
//}

//suan'shu'ping'jun'lv'bo
u8 TCS34725_GetRawData(COLOR_RGBC *rgbc, int num_measurements)
{
    u8 status = TCS34725_STATUS_AVALID;
//    int i;
//    uint16_t c_accumulator = 0, r_accumulator = 0, g_accumulator = 0, b_accumulator = 0;
//    
//    for (i = 0; i < num_measurements; ++i) {
//        TCS34725_Read(TCS34725_STATUS, &status, 1);
//        
//        if (status & TCS34725_STATUS_AVALID) {
//            c_accumulator += TCS34725_GetChannelData(TCS34725_CDATAL);
//            r_accumulator += TCS34725_GetChannelData(TCS34725_RDATAL);
//            g_accumulator += TCS34725_GetChannelData(TCS34725_GDATAL);
//            b_accumulator += TCS34725_GetChannelData(TCS34725_BDATAL);
//        } else {
//            // Handle invalid status if needed
//            return 0;
//        }
//    }
//    rgbc->c = c_accumulator / num_measurements;
//    rgbc->r = r_accumulator / num_measurements;
//    rgbc->g = g_accumulator / num_measurements;
//    rgbc->b = b_accumulator / num_measurements;
    
	TCS34725_Read(TCS34725_STATUS, &status, 1);
	if (status & TCS34725_STATUS_AVALID) 
	{
		rgbc->c = TCS34725_GetChannelData(TCS34725_CDATAL);
		rgbc->r = TCS34725_GetChannelData(TCS34725_RDATAL);
		rgbc->g = TCS34725_GetChannelData(TCS34725_GDATAL);
		rgbc->b = TCS34725_GetChannelData(TCS34725_BDATAL);
    }
	else
	{
        return 0;
    }
	
	
	
	
    return 1;
}

/******************************************************************************/
//RGBתHSL
void RGBtoHSL(COLOR_RGBC *Rgb, COLOR_HSL *Hsl)
{
	u8 maxVal,minVal,difVal;
	u8 r = Rgb->r*100/Rgb->c;   //[0-100]
	u8 g = Rgb->g*100/Rgb->c;
	u8 b = Rgb->b*100/Rgb->c;
	
	maxVal = max3v(r,g,b);
	minVal = min3v(r,g,b);
	difVal = maxVal-minVal;
	
	//��������
	Hsl->l = (maxVal+minVal)/2;   //[0-100]
	
	if(maxVal == minVal)//��r=g=b,�Ҷ�
	{
		Hsl->h = 0; 
		Hsl->s = 0;
	}
	else
	{
		//����ɫ��
		if(maxVal==r)
		{
			if(g>=b)
				Hsl->h = 60*(g-b)/difVal;
			else
				Hsl->h = 60*(g-b)/difVal+360;
		}
		else
			{
				if(maxVal==g)Hsl->h = 60*(b-r)/difVal+120;
				else
					if(maxVal==b)Hsl->h = 60*(r-g)/difVal+240;
			}
		
		//���㱥�Ͷ�
		if(Hsl->l<=50)Hsl->s=difVal*255/(maxVal+minVal);  //[0-100]
		else
			Hsl->s=difVal*100/(200-(maxVal+minVal));
	}
}
/******************************************************************************/

//����RGB�ı�����Lux
void RGBto255RGB(COLOR_RGBC *Rgb,Sepan_RGBC *RGB255)
{
	double maxVal,minVal,difVal;
	double r_255=0.0,g_255=0.0,b_255=0.0;

	r_255 = (double)Rgb->r/Rgb->c*255;
	g_255 = (double)Rgb->g/Rgb->c*255;
	b_255 = (double)Rgb->b/Rgb->c*255;
	
	maxVal = max3v(r_255,g_255,b_255);
	r_255 = r_255/maxVal*255;
	g_255 = g_255/maxVal*255;
	b_255 = b_255/maxVal*255;
	
//	maxVal = max3v(Rgb->r,Rgb->g,Rgb->b);	
//	r_255 = (double)Rgb->r/maxVal*255;
//	g_255 = (double)Rgb->g/maxVal*255;
//	b_255 = (double)Rgb->b/maxVal*255;
	
	RGB255->r = (unsigned char)r_255;
	RGB255->g = (unsigned char)g_255;
	RGB255->b = (unsigned char)b_255;
	lux=(0.299*Rgb->r)+(0.587*Rgb->g)+(0.114*Rgb->b);
	RGB255->Lux = (unsigned short)lux;
	
	if(RGB255->r == 255 && RGB255->g == 255 && RGB255->b == 255)
	{
		if(Rgb->c<255)
		{
			RGB255->r = 0;
			RGB255->g = 0;
			RGB255->b = 0;
		}
	}
}
//
//����CCTɫ��
double calculateColorTemperature(COLOR_RGBC *Rgb)
{
	double trimX = 0;
  double trimY = 0;
  double trimZ = 0;
  double coorX = 0, coorY = 0;
  double CCT = 0;
  double n = 0;
  int R = Rgb->r;//255;
  int G = Rgb->g;//231;
  int B = Rgb->b;//131;

	//���¹�ʽʵ��RGBת���̼�ֵ
	trimX = 2.789 * R + 1.7517 * G + 1.1302 * B;
	trimY = 1 * R + 4.5907 * G + 0.0601 * B;
	trimZ = 0 * R + 0.0565 * G + 5.5943 * B;
	//���¹�ʽʵ�����̼�ֵתɫ����
	coorX = trimX / (trimX + trimY + trimZ);
	coorY = trimY / (trimX + trimY + trimZ);
	n = (coorX - 0.3320) / (0.1858 - coorY);
	//���¹�ʽʵ��ɫ����תɫ��
	CCT = 437 * n * n * n + 3601 * n * n + 6831 * n + 5517;
	return CCT;
}


