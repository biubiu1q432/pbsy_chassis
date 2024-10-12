#include "stm32f10x.h"  // Device header
#include "freertos.h"
#include "task.h"
 
#include "stdarg.h" 
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "Delay.h"
#include "LED.h"
#include "usart.h"

#include "HallEncoder.h"
#include "Key.h"
#include "OLED.h"
#include "PWM.h"
#include "Servo.h"

#include "tcs34725.h"
#include "timer.h"

#include "rc522_config.h"
#include "rc522_function.h"

#include "VL6180.h"

#include "queue.h"




#define pi 			3.14
#define Encoder_N	28	//每圈脉冲数
#define Encoder_K	100	//齿比
#define FRE			100	//采样频率（Hz）
#define Wheel_spacing	0.09




TaskHandle_t  motor_handler;//任务句柄
TaskHandle_t  motion_handler;//任务句柄
TaskHandle_t  basic_handler;//任务句柄
TaskHandle_t  MPU_handler;//任务句柄
TaskHandle_t  Master_handler;//任务句柄
TaskHandle_t  Mission_handler_handler;//任务句柄
TaskHandle_t  tcs_handler;//颜色传感器任务句柄
TaskHandle_t  rcc_handler;//读卡器任务句柄
TaskHandle_t  Lazer_handler;//激光传感器句柄

TaskHandle_t  Sender_handler;

void motor_task(void *pvParameters);//函数声明
void motion_task(void *pvParameters);//函数声明
void basic_task(void *pvParameters);
void MPU_task(void *pvParameters);
void Master_task(void *pvParameters);
void Mission_handler(void *pvParameters);
void tcs_task(void *pvParameters);
void rcc_task(void *pvParameters);
void Lazer_task(void *pvParameters);

void Sender_task(void *pvParameters);

COLOR_RGBC rgb;
COLOR_HSL  hsl;
Sepan_RGBC rgb255;

int previous_color ;

//读卡器  
unsigned char status,Result;
unsigned int temp,i;
unsigned char  data1[16] = {0x12,0x34,0x56,0x78,0xED,0xCB,0xA9,0x87,0x12,0x34,0x56,0x78,0x04,~0x04,0x04,~0x04};
unsigned char  data2[4]  = {0,0,0,1};
unsigned char  DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
unsigned char g_ucTempbuf[20]; 


#define N1 2
#define N2 2
#define N3 2
#define N4 2

uint16_t filter1(uint16_t x) 
{
    static uint16_t buffer[N1] = {0};
    static uint16_t sum = 0;
    static int ptr = 0;
    sum = sum - buffer[ptr] + x;
    buffer[ptr] = x;
    ptr = (ptr + 1) % N1;
    return sum / N1;
}
uint16_t filter2(uint16_t x) 
{
    static uint16_t buffer[N2] = {0};
    static uint16_t sum = 0;
    static int ptr = 0;
    sum = sum - buffer[ptr] + x;
    buffer[ptr] = x;
    ptr = (ptr + 1) % N2;
    return sum / N2;
}
uint16_t filter3(uint16_t x) 
{
    static uint16_t buffer[N3] = {0};
    static uint16_t sum = 0;
    static int ptr = 0;
    sum = sum - buffer[ptr] + x;
    buffer[ptr] = x;
    ptr = (ptr + 1) % N3;
    return sum / N3;
}
uint16_t filter4(uint16_t x) 
{
    static uint16_t buffer[N4] = {0};
    static uint16_t sum = 0;
    static int ptr = 0;
    sum = sum - buffer[ptr] + x;
    buffer[ptr] = x;
    ptr = (ptr + 1) % N4;
    return sum / N4;
}

typedef struct//激光结构体
{
	int16_t Front;
	int16_t Left_Front;
	int16_t Left_Back;
	int16_t Right_Front;
	int16_t Right_Back;
	
	int16_t Left_Front_Offset;
	int16_t Left_Back_Offset;
	int16_t Right_Front_Offset;
	int16_t Right_Back_Offset;	
	
	int16_t Left_Average;
	int16_t Right_Average;	
	
	int16_t Factor;
}Lazer_Structure;
Lazer_Structure Lazer;

typedef struct//欧拉角结构体
{

int16_t aacz,yaw,GYO;				//偏航角
int16_t yaw_offset;	//校准值
int16_t yaw_Target;	//校准值
uint8_t yaw_offset_get;	
int16_t err;
int16_t adjust;
int16_t adjusterr;
}Euler_Structure;
Euler_Structure Euler;

typedef struct//MISSION结构体
{
	uint8_t Remote_Flag;//是否有遥控介入
	uint8_t state;
	uint8_t num;
	uint8_t VZ_lock;
	uint8_t timer;
	float parameter1;
	int16_t parameter2;
	int16_t parameter3;
}Mission_Structure;
Mission_Structure Mission;

typedef struct//底盘状态结构体
{
	float AngleA;float AngleB;
	float Last_AngleA_SPEEDCAL;float Last_AngleB_SPEEDCAL;
	float Last_AngleA;float Last_AngleB;
	float SpeedA;float SpeedB;
	float TargetA;float TargetB;
	float Vx;float Vz;
	float VxTarget;float VyTarget;float VzTarget;
	float VxFilter;
	float Adjust;
	
	uint8_t Accumulation_X_Flag;
	float Accumulation_X;
	float Last_Accumulation_X;
		
	uint16_t Angle;
}Car_Motion_Structure;
Car_Motion_Structure Car_Motion;

extern uint8_t _OUTPUT_USART;		//用于print的串口号

typedef struct//串口结构体
{
	uint8_t Rx_String_Copy[50];
	char * Rx_cut;
//	int16_t Buf[3];
//	float F_Buf[3];
	uint8_t Rx_Check;
	uint8_t RX_OK;
	uint8_t Rx_String[50];    			//接收字符串数组
	uint8_t Rx_Flag;         			//接收字符串计数
	uint8_t Rx_buff;     				//接收缓存
}Rx_Structure;
Rx_Structure MPU;
Rx_Structure Master;
Rx_Structure LazerRX;

struct messageQueue{
	int id;
	char msg[100];
};

QueueHandle_t xMyQueueHandle;

uint8_t Light_Switch = 1;
uint8_t Servo_Angle = 90;

    static uint8_t d10;
//  struct messageQueue msg;
	struct messageQueue _sData;
	struct messageQueue*sData = &_sData;

void FOC_ANGLE_SET(uint8_t moter,float angle);
void Mission_manager(uint16_t NUM,float SPEED,uint16_t PA,uint16_t PB);
void RealSpeed(void);
void offset_Init(void);

CanRxMsg RxMessage;

void CAN_Mode_Init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);//使能PORTA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
    GPIO_Init(GPIOA, &GPIO_InitStructure);		//初始化IO
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化IO
    //CAN单元设置
    CAN_InitStructure.CAN_TTCM=DISABLE;						 //非时间触发通信模式  //
    CAN_InitStructure.CAN_ABOM=ENABLE;						 //软件自动离线管理	 //
    CAN_InitStructure.CAN_AWUM=ENABLE;						 //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)//
    CAN_InitStructure.CAN_NART=ENABLE;						 	//禁止报文自动传送 //
    CAN_InitStructure.CAN_RFLM=ENABLE;						 //报文不锁定,新的覆盖旧的 //
    CAN_InitStructure.CAN_TXFP=DISABLE;						 //优先级由报文标识符决定 //
    CAN_InitStructure.CAN_Mode= 0;	         //模式设置： mode:0,普通模式;1,回环模式; //
    //设置波特率
    CAN_InitStructure.CAN_SJW=tsjw;				//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
    CAN_InitStructure.CAN_BS1=tbs1; //Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
    CAN_InitStructure.CAN_BS2=tbs2;//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
    CAN_InitStructure.CAN_Prescaler=brp;            //分频系数(Fdiv)为brp+1	//
    CAN_Init(CAN1, &CAN_InitStructure);            // 初始化CAN1
    CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位
    CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;//32位ID
    CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;	//32位MASK
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
    CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
}
uint8_t Can_Send_Msg(uint8_t* msg,uint8_t len)
{
    uint16_t i=0;
    uint8_t mbox;
    CanTxMsg TxMessage;
    TxMessage.StdId=0x13;			// 标准标识符
    TxMessage.ExtId=0x12;			// 设置扩展标示符
    TxMessage.IDE=CAN_Id_Standard; // 标准帧
    TxMessage.RTR=CAN_RTR_Data;		 // 数据帧
    TxMessage.DLC=len;						// 要发送的数据长度
    for(i=0; i<len; i++)
        TxMessage.Data[i]=msg[i];
    mbox= CAN_Transmit(CAN1, &TxMessage);
    i=0;
    while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))
        i++;	//等待发送结束
    if(i>=0XFFF)
	{
        return 1;
	}
    return 0;
}
uint8_t Can_Receive_Msg(uint8_t *buf)
{
    u32 i;
	uint16_t TIMEOUT = 4;
	
	while(TIMEOUT)
	{
		if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)
		{
			vTaskDelay(1);
			TIMEOUT --;
		}
		else
		{
			TIMEOUT = 0;
		}
		if(TIMEOUT == 1)
		{
			return 0;		//没有接收到数据,退出
		}
	}
	
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据
    for(i=0; i<8; i++)
        buf[i]=RxMessage.Data[i];
    return RxMessage.DLC;
}

float target_limit_float(float insert,float low,float high)//浮点数限幅
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
void Speed_Get(uint8_t Moter)//获取电机角度
{	
	uint8_t canbuf[8] = {0};
	
	Can_Receive_Msg(canbuf);
	
	if(canbuf[1] == 1)
	{
		Car_Motion.AngleA = 100 * canbuf[3] + canbuf[4] + 0.01 * canbuf[5];
	}
	else if(canbuf[1] == 2)
	{
		Car_Motion.AngleB = 100 * canbuf[3] + canbuf[4] + 0.01 * canbuf[5];
	}
	
	
//	if(Moter == 1)
//	{
//		Car_Motion.AngleA = 100 * canbuf[3] + canbuf[4] + 0.01 * canbuf[5];
//		if(counter[0]>=5){printf("LEFT %d %d %d\n",canbuf[3],canbuf[4],canbuf[5]);counter[0] = 0;}
//		//Car_Motion.SpeedA = Car_Motion.AngleA * FRE * (pi * 0.044) / Encoder_N / Encoder_K;
//		
//	}
//	else if(Moter == 2)
//	{
//		if(counter[1]>=5){printf("RIGHT %d %d %d\n",canbuf[3],canbuf[4],canbuf[5]);counter[1] = 0;}
//		Car_Motion.AngleB = 100 * canbuf[3] + canbuf[4] + 0.01 * canbuf[5];
//		//Car_Motion.SpeedB = Car_Motion.AngleA * FRE * (pi * 0.044) / Encoder_N / Encoder_K;
//	}
}
void Speed_Cul(void)//解算车轮目标速度
{	
	static uint16_t Filter_Divider;
	float omega;
	omega = Car_Motion.VzTarget;	
	
//	Filter_Divider ++;
//	if(Filter_Divider >= 10)//10HZ
//	{
//		Filter_Divider = 0;
//		Car_Motion.VxFilter += (Car_Motion.VxTarget - Car_Motion.VxFilter)*0.8f;
//	}
	
	Car_Motion.VxFilter = Car_Motion.VxTarget;
	
	Car_Motion.TargetA = Car_Motion.VxFilter-omega*(Wheel_spacing/2);
	Car_Motion.TargetB = -(Car_Motion.VxFilter+omega*(Wheel_spacing/2));
	
	Car_Motion.SpeedA = (Car_Motion.AngleA - Car_Motion.Last_AngleA_SPEEDCAL)* FRE * 0.027; //0.3
	Car_Motion.SpeedB = (Car_Motion.AngleB - Car_Motion.Last_AngleB_SPEEDCAL)* FRE * 0.027;
	
	RealSpeed();
	
	Car_Motion.Last_AngleA_SPEEDCAL = Car_Motion.AngleA;
	Car_Motion.Last_AngleB_SPEEDCAL = Car_Motion.AngleB;
}
void PID_Set(uint8_t Moter)//设置轮速
{
	if(Moter == 1)
	{
		FOC_ANGLE_SET(Moter,Car_Motion.Last_AngleA + Car_Motion.TargetA/2.65);
		Car_Motion.Last_AngleA = Car_Motion.Last_AngleA + Car_Motion.TargetA/2.65;
	}
	else if(Moter == 2)
	{
		FOC_ANGLE_SET(Moter,Car_Motion.Last_AngleB + Car_Motion.TargetB/2.65);
		Car_Motion.Last_AngleB = Car_Motion.Last_AngleB + Car_Motion.TargetB/2.65;
	}
}
void RealSpeed(void)
{
	Car_Motion.Vx = (Car_Motion.SpeedA+Car_Motion.SpeedB)/2.0f;//Car_Motion.Vx = (Car_Motion.SpeedA+Car_Motion.SpeedB)/2.0f;
	Car_Motion.Vz = (Car_Motion.SpeedA-Car_Motion.SpeedB)/Wheel_spacing;
}

void User_Init()
{
	delay_ms(100);//1000
	
	LED_Init();
	Serial_Init();
	printf("restart\n");//  \r
	TCS34725_Init();
	RC522_Init();
	RC522_SPI_Config ();
	
	VL6180_GPIO_Config();
	VL6180X_Init(1);
	VL6180X_Init(2);
	VL6180X_Init(3);
	VL6180X_Init(4);

	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_4tq,4,1);
	
	delay_ms(100);
}
void Value_init()
{
	Mission.Remote_Flag = 0;
	Mission.num = 0;
	Mission.state = 0;
	Euler.yaw = 0;
	Euler.yaw_offset = 0;
	Euler.yaw_offset_get = 0;
	Euler.adjust = 0;
	Euler.yaw_Target = 0;
//	Car_Motion.VxTarget = 0.1;
//	Car_Motion.VzTarget = 1.57;
//	Mission.state = 1;
//	Mission.num = 1;
}
int main()
{
	User_Init();
	Value_init();
	offset_Init();
	xMyQueueHandle = xQueueCreate(20,sizeof(struct messageQueue));

//    if(xMyQueueHandle == NULL)
//    {
//        // 队列创建失败
//        while(1);
//    }
	
	xTaskCreate( motion_task,"motion_task",256,NULL,4,&motion_handler );
	xTaskCreate( motor_task,"motor_task",256,NULL,5,&motor_handler );
	xTaskCreate( basic_task,"basic_task",128,NULL,1,&basic_handler );
	xTaskCreate( MPU_task,"MPU_task",256,NULL,3,&MPU_handler );
	xTaskCreate( Master_task,"Master_task",128,NULL,2,&Master_handler );
	xTaskCreate( Mission_handler,"Mission_handler",256,NULL,2,&Mission_handler_handler);
	xTaskCreate( tcs_task,"tcs_task",128,NULL,1,&tcs_handler );
	xTaskCreate( rcc_task,"rcc_task", 128,NULL,1,&rcc_handler );
	xTaskCreate( Lazer_task,"Lazer_task",256,NULL,1,&Lazer_handler );	
	xTaskCreate( Sender_task,"Sender_task",128,NULL,4,&Sender_handler );

	vTaskStartScheduler(); //开始调度
}

void motion_task(void *pvParameters)
{
	while(1)
	{
		Speed_Cul();
		vTaskDelay(10);
	}
}
void motor_task(void *pvParameters)//单个轮子频率为200Hz
{
	while(1)
	{
		PID_Set(1);
		Speed_Get(1);
		vTaskDelay(5);
		PID_Set(2);
		Speed_Get(2);
		vTaskDelay(5);
		if(Car_Motion.Accumulation_X_Flag == 1)
		{
			Car_Motion.Accumulation_X += Car_Motion.Vx ;

		}
	}
}
void MPU_task(void *pvParameters)
{
	float Alpha;
	static int errI;
	static int Lasterr;
	float Angle_P,Angle_I,Angle_D,Angle_Q = 0.001;
	while(1)
	{
		if(!Mission.Remote_Flag)
		{

			Euler.err = Euler.yaw_offset - Euler.yaw - Euler.yaw_Target;
			Euler.adjusterr = Euler.err - Euler.adjust;
			
			errI += Euler.adjusterr;
			errI = target_limit_float(errI,-5000,5000);	
			
			Alpha = 0;
			if((Car_Motion.Vx < 0.2) && (Car_Motion.Vx > -0.2))//(Angle_Adjustment_ON == 1)
			{
				Angle_P = 0.0004;Angle_I = 0.00001;Angle_D = -0.0010;//静止
			}
			else
			{
				Angle_P = 0.0000;Angle_I = 0.00003;Angle_D = -0.0003;//运动
			}
			if((Euler.adjusterr < 300) && (Euler.adjusterr > -300))
			{
				errI = 0;
			}
			Alpha = Euler.adjusterr * Angle_P + errI * Angle_I + (Euler.adjusterr - Lasterr) * Angle_D;// + errI * Angle_Q;
			Alpha = target_limit_float(Alpha, -1,1);
			Lasterr = Euler.adjusterr;
			Car_Motion.VzTarget = Alpha;
		}
		vTaskDelay(10);
	}
}
void Master_task(void *pvParameters)
{
	while(1)
	{
		if(Master.RX_OK)
		{
			uint16_t BUF;
			float BUF_SPEED;
			uint16_t BUF_A,BUF_B;
			
			Master.Rx_cut = strtok(Master.Rx_String_Copy, "|");
			Master.Rx_cut = strtok(NULL, "|");
			BUF = atoi(Master.Rx_cut);
			if(BUF == 66)
			{
				Mission.Remote_Flag = 1;
				Master.Rx_cut = strtok(NULL, "|");
				Car_Motion.VxTarget = atof(Master.Rx_cut);
				Master.Rx_cut = strtok(NULL, "|");
				Car_Motion.VyTarget = atof(Master.Rx_cut);
				Master.Rx_cut = strtok(NULL, "|");
				Car_Motion.VzTarget = atof(Master.Rx_cut);				
			}
			else
			{
				Mission.Remote_Flag = 0;
				Master.Rx_cut = strtok(NULL, "|");
				BUF_SPEED = atoi(Master.Rx_cut);
				Master.Rx_cut = strtok(NULL, "|");
				BUF_A = atoi(Master.Rx_cut);
				Master.Rx_cut = strtok(NULL, "|");
				BUF_B = atoi(Master.Rx_cut);
				
				if(BUF == 6)
				{
					Car_Motion.Accumulation_X_Flag = 1;
					Car_Motion.Accumulation_X = 0;
				}
				Mission_manager(BUF,BUF_SPEED,BUF_A,BUF_B);
			}
		}
		Master.RX_OK = 0;
		vTaskDelay(1);
	}
}

void Mission_manager(uint16_t NUM,float SPEED,uint16_t PA,uint16_t PB)
{
	if(Mission.state == 0)
	{
		Mission.num = NUM;
		Mission.parameter1 = SPEED;
		Mission.parameter2 = PA;//输入的距离
		Mission.parameter3 = PB;
		Mission.state = 1;
		printf("GOT\n");
	}
	else if(NUM == 5)
	{
		Mission.num = NUM;
		Mission.parameter1 = SPEED;
		Mission.parameter2 = PA;
		Mission.parameter3 = PB;
		Mission.state = 1;
		printf("GOT\n");
	}
	else
	{
		printf("UNGOT\n");
	}
	
}

void Mission_handler(void *pvParameters)
{
//10ms检查一次任务请求及完成情况
//	static uint16_t SPEEDUP_COUNTER = 0;
	while(1)
	{
//		SPEEDUP_COUNTER ++;
		if(Mission.num == 1)
		{
			float Front_DIS;
			int16_t LEFT_DIS;
			//直行对正
			//parameter1：对正最大速度//parameter2：前方停止距离//parameter3：中心死区宽度
			Front_DIS = Lazer.Front - Mission.parameter2;
			//if((Euler.err <500) && (Euler.err >-500))//如果当前车身姿态正确
//			{
				if(Front_DIS >=  100)//如果未靠近目标(不包括负距离)
				{
					if(Lazer.Left_Average < Mission.parameter3 / 2)//如果过于靠近左侧边缘
					{
						printf("TOO LEFT\n");
						Car_Motion.VxTarget = 0;
						vTaskDelay(250);//while(Car_Motion.Vx > 0.01){;}
						Euler.adjust = 2000;//右偏
						vTaskDelay(200);
						while((Euler.adjusterr >500) || (Euler.adjusterr <-500)){vTaskDelay(100);}
						Car_Motion.VxTarget = 0.05;
						vTaskDelay(1000);
						Car_Motion.VxTarget = 0;
						vTaskDelay(250);//while(Car_Motion.Vx > 0.01){;}	
						Euler.adjust = 0;
						vTaskDelay(200);	
						while((Euler.adjusterr >500) || (Euler.adjusterr <-500)){vTaskDelay(100);}
//						printf("L:%f R:%f\n",Car_Motion.SpeedA,Car_Motion.SpeedB);
//						printf("H:%f\n",Car_Motion.Vx);
//						printf("LA:%d RA:%d\n", Lazer.Left_Average, Lazer.Right_Average);
						goto line1;
					}
					else if(Lazer.Right_Average < Mission.parameter3 / 2)//如果过于右侧靠近边缘
					{
						printf("TOO RIGHT\n");
						Car_Motion.VxTarget = 0;
						vTaskDelay(250);//while(Car_Motion.Vx > 0.01){;}
						Euler.adjust = -2000;//左偏
						vTaskDelay(200);
						while((Euler.adjusterr >500) || (Euler.adjusterr <-500)){vTaskDelay(100);}
						Car_Motion.VxTarget = 0.05;
						vTaskDelay(1000);
						Car_Motion.VxTarget = 0;
						vTaskDelay(250);//while(Car_Motion.Vx > 0.01){;}	
						Euler.adjust = 0;
						vTaskDelay(200);	
						while((Euler.adjusterr >500) || (Euler.adjusterr <-500)){vTaskDelay(100);}
//						printf("L:%f R:%f\n",Car_Motion.SpeedA,Car_Motion.SpeedB);
//						printf("H:%f\n",Car_Motion.Vx);
//						printf("LA:%d RA:%d\n", Lazer.Left_Average, Lazer.Right_Average);
						goto line1;
					}
					else if(Lazer.Left_Average < Mission.parameter3)//左偏	//如果偏离小
					{
						Euler.adjust = 1000;
						vTaskDelay(200);
						printf("too left\n");
//						printf("L:%f R:%f\n",Car_Motion.SpeedA,Car_Motion.SpeedB);
//						printf("H:%f\n",Car_Motion.Vx);
//						printf("LA:%d RA:%d\n", Lazer.Left_Average, Lazer.Right_Average);
					}					
					else if(Lazer.Right_Average < Mission.parameter3)//右偏
					{
						Euler.adjust = -1000;
						vTaskDelay(200);
						printf("too right\n");
//						printf("L:%f R:%f\n",Car_Motion.SpeedA,Car_Motion.SpeedB);
//						printf("H:%f\n",Car_Motion.Vx);
//						printf("LA:%d RA:%d\n", Lazer.Left_Average, Lazer.Right_Average);
					}
					else
					{
						Euler.adjust = 0;
					}
				}
				else
				{
					Euler.adjust = 0;
				}
			    if(Front_DIS <= 0)
				{
					Car_Motion.VxTarget = -0.03;
				}
				else
				{
					if(Front_DIS <= 10)
					{
						Car_Motion.VxTarget = 0;
						Mission.state = 0;
						Mission.num = 0;
						Car_Motion.Accumulation_X_Flag = 0;
//						printf("DONE\n");
						sprintf((char *)_sData.msg, "@\n");
			      xQueueSend(xMyQueueHandle, &_sData, 0);
					}
					else if(Front_DIS <= 1.2*Mission.parameter2)
					{
						Car_Motion.VxTarget = Mission.parameter1*0.001;
//						printf("L:%f R:%f\n",Car_Motion.SpeedA,Car_Motion.SpeedB);
//						printf("H:%f\n",Car_Motion.Vx);
					}
					else if(Front_DIS <= 1.8*Mission.parameter2)
					{
						Car_Motion.VxTarget = Mission.parameter1*0.003;
//						printf("L:%f R:%f\n",Car_Motion.SpeedA,Car_Motion.SpeedB);
//						printf("H:%f\n",Car_Motion.Vx);
					}
					else if(Front_DIS <= 2.5*Mission.parameter2)
					{
						Car_Motion.VxTarget =Mission.parameter1*0.005;
//						printf("L:%f R:%f\n",Car_Motion.SpeedA,Car_Motion.SpeedB);
//						printf("H:%f\n",Car_Motion.Vx);
					}
					else
					{
						Car_Motion.VxTarget = Mission.parameter1*0.01;
//						printf("L:%f R:%f\n",Car_Motion.SpeedA,Car_Motion.SpeedB);
//						printf("H:%f\n",Car_Motion.Vx);
					}
					line1:	{}				
				}
//			}
		}
		else if(Mission.num == 2)
		{
			if(Mission.parameter1 == 0)
			{
				Euler.yaw_Target = Mission.parameter2 * 182;
			}
			else if(Mission.parameter1 == 1)
			{
				int16_t temp = Euler.yaw_Target;
				temp += Mission.parameter2 * 182;
				if(temp < -32768)
				{
					temp += 65536;
				}
				else if(temp > 32768)
				{
					temp -= 65536;
				}
				Euler.yaw_Target = temp;
				Mission.parameter1 = 2;
			}
			vTaskDelay(200);
			if((Euler.err < 500) && (Euler.err > -500))
			{
				vTaskDelay(600);
//				printf("DONE\n");
				sprintf((char *)_sData.msg, "@\n");
			    xQueueSend(xMyQueueHandle, &_sData, 0);
				Mission.state = 0;
				Mission.num = 0;
			}
			
		}
		else if(Mission.num == 3)
		{
			int16_t offset[2];
			int16_t err;
			uint8_t found = 0;
			Euler.adjust = -4000;

			vTaskDelay(200);
			while((Euler.adjusterr >200) || (Euler.adjusterr <-200)){vTaskDelay(100);}
			printf("-1\n");
			vTaskDelay(200);
			Euler.adjust = 2500;//从左到右
			vTaskDelay(200);
			while((Euler.adjusterr >200) || (Euler.adjusterr <-200))
			{	
				err = Lazer.Left_Front - Lazer.Left_Back;
				if((err <= 5) && (err >= -5) && (found == 0))
				{
					found = 1;
					offset[0] = Euler.yaw - Euler.yaw_Target;
					printf("-2\n");
				}
				vTaskDelay(100);
			}
			vTaskDelay(200);
			Euler.adjust = -2500;//从右到左
			vTaskDelay(200);
			found = 0;
			while((Euler.adjusterr >200) || (Euler.adjusterr <-200))
			{	
				err = Lazer.Left_Front - Lazer.Left_Back;
				if((err <= 5) && (err >= -5) && (found == 0))
				{
					found = 1;
					offset[1] = Euler.yaw - Euler.yaw_Target;
					printf("-3\n");
				}
				vTaskDelay(100);
			}
			vTaskDelay(200);
			Euler.yaw_offset = (offset[0] + offset[1]) / 2;
			Euler.adjust = 0;
			while((Euler.adjusterr >200) || (Euler.adjusterr <-200)){vTaskDelay(100);}
			vTaskDelay(600);
//			printf("DONE\n");
			sprintf((char *)_sData.msg, "@\n");
			xQueueSend(xMyQueueHandle, &_sData, 0);
			Mission.state = 0;
			Mission.num = 0;
			
		}
		else if(Mission.num == 5)
		{
			Car_Motion.Accumulation_X_Flag = 0;
			Car_Motion.VxTarget = 0;
			Euler.adjust = 0;
			Mission.state = 0;
			Mission.num = 0;
			printf("DONE5\n");
		}
		else if(Mission.num == 6)
		{
			int16_t Front_DIS;
			int16_t LEFT_DIS;
			//直行对正
			//parameter1：对正最大速度//parameter2：前方停止距离//parameter3：中心死区宽度
			
			
			Front_DIS = Mission.parameter2 - Car_Motion.Accumulation_X;
			
			sprintf((char *)_sData.msg, "$|%f|\n", Car_Motion.Accumulation_X); 
			xQueueSend(xMyQueueHandle, &_sData, 0);
			
			
			if(Front_DIS >=  0.1*Mission.parameter2)//如果未靠近目标(不包括负距离)
			{
				if(Lazer.Left_Average < Mission.parameter3 / 2)//如果过于靠近左侧边缘
				{
					printf("TOO LEFT\n");
					Car_Motion.VxTarget = 0;
					vTaskDelay(250);//while(Car_Motion.Vx > 0.01){;}
					Euler.adjust = 2000;//右偏
					vTaskDelay(200);
					while((Euler.adjusterr >500) || (Euler.adjusterr <-500)){vTaskDelay(100);}
					Car_Motion.VxTarget = 0.05;
					vTaskDelay(1000);
					Car_Motion.VxTarget = 0;
					vTaskDelay(250);//while(Car_Motion.Vx > 0.01){;}	
					Euler.adjust = 0;
					vTaskDelay(200);	
					while((Euler.adjusterr >500) || (Euler.adjusterr <-500)){vTaskDelay(100);}
//					printf("%f\n", Car_Motion.Accumulation_X); 
					goto line1;
				}
				else if(Lazer.Right_Average < Mission.parameter3 / 2)//如果过于右侧靠近边缘
				{
					printf("TOO RIGHT\n");
					Car_Motion.VxTarget = 0;
					vTaskDelay(250);//while(Car_Motion.Vx > 0.01){;}
					Euler.adjust = -2000;//左偏
					vTaskDelay(200);
					while((Euler.adjusterr >500) || (Euler.adjusterr <-500)){vTaskDelay(100);}
					Car_Motion.VxTarget = 0.05;
					vTaskDelay(1000);
					Car_Motion.VxTarget = 0;
					vTaskDelay(250);//while(Car_Motion.Vx > 0.01){;}	
					Euler.adjust = 0;
					vTaskDelay(200);	
					while((Euler.adjusterr >500) || (Euler.adjusterr <-500)){vTaskDelay(100);}
//					printf("%f\n", Car_Motion.Accumulation_X); 
					goto line2;
				}
				else if(Lazer.Left_Average < Mission.parameter3)//左偏	//如果偏离小
				{
					Euler.adjust = 1000;
					vTaskDelay(200);
					printf("too left\n");
					
				}					
				else if(Lazer.Right_Average < Mission.parameter3)//右偏
				{
					Euler.adjust = -1000;
					vTaskDelay(200);
					printf("too right\n");
					
				}
				else
				{
					Euler.adjust = 0;
				 
				}
			}
			else
			{
				Euler.adjust = 0;
				 
			}
			if(Front_DIS <= 0)
			{
				Car_Motion.VxTarget = -0.04;
			
			}
			else
			{
				if(Front_DIS <= 1)
				{
					Car_Motion.VxTarget = 0;
					Mission.state = 0;
					Mission.num = 0;
//					printf("DONE\n");
					sprintf((char *)_sData.msg, "@\n");
					xQueueSend(xMyQueueHandle, &_sData, 0);
			   
				}
				else if(Front_DIS <= 0.1*Mission.parameter2 | Front_DIS <= 20)
				{
					Car_Motion.VxTarget = Mission.parameter1*0.003;
					
				}
				else if(Front_DIS <= 0.3*Mission.parameter2)
				{
					Car_Motion.VxTarget = Mission.parameter1*0.005;
				
				}
				else if(Front_DIS <= 0.5*Mission.parameter2)
				{
					Car_Motion.VxTarget = Mission.parameter1*0.008;
					
				}
				else
				{
					Car_Motion.VxTarget = Mission.parameter1*0.01;
				}
				line2:	{ }				
			}

		}
	

		vTaskDelay(10);
	}
}

void basic_task(void *pvParameters)
{
//    static uint8_t d10;
////    struct messageQueue msg;
//	struct messageQueue _sData;
//	struct messageQueue*sData = &_sData;
//	while(1)
//	{
//		sprintf((char *)_sData.msg,"1234");
//		
//		sprintf((char *)_sData.msg,"3456");
//		xQueueSend( xMyQueueHandle,&_sData,0 );
//		vTaskDelay(200);
//	}
    
    while(1)
    {
        d10++;
		
        vTaskDelay(100 / portTICK_PERIOD_MS);  // 延时 100ms

        if (d10 >= 5)
        {
            LED_Turn();  // 控制 LED 翻转状态
            
            // 使用 sprintf 格式化数据并存入消息队列
//             sprintf((char *)_sData.msg, "LA:%d RA:%d\n", Lazer.Left_Average, Lazer.Right_Average);
//			 sprintf((char *)_sData.msg, "F:%d", Lazer.Front);
//			 xQueueSend(xMyQueueHandle, &_sData, 0);
//			 vTaskDelay(10);
//			 sprintf((char *)_sData.msg(char *)_sData.msg, "L:%f R:%f", Car_Motion.AngleA, Car_Motion.AngleB);
			 sprintf((char *)_sData.msg, "LF:%d LB:%d",Lazer.Left_Front,Lazer.Left_Back);
			 xQueueSend(xMyQueueHandle, &_sData, 0);
			 sprintf((char *)_sData.msg, "RF:%d RB:%d\n", Lazer.Right_Front, Lazer.Right_Back);
			 xQueueSend(xMyQueueHandle, &_sData, 0);
//			
//			 sprintf((char *)_sData.msg, "\nYAW:%d", Euler.yaw);
//			 sprintf((char *)_sData.msg, "%f\n", Car_Motion.Accumulation_X); 
//				xQueueSend(xMyQueueHandle, &_sData, 0);
//			sprintf((char *)_sData.msg,"H:%f\n",Car_Motion.Vx);
//			xQueueSend(xMyQueueHandle, &_sData, 0);
					
					
			 sprintf((char *)_sData.msg, "#%d|%d|%d|\n",hsl.h,hsl.s,hsl.l);
			 xQueueSend(xMyQueueHandle, &_sData, 0);
//			
//			 sprintf((char *)_sData.msg, "#%d|%d|%d|\n",rgb.r,rgb.g,rgb.b);
//			 xQueueSend(xMyQueueHandle, &_sData, 0);
//			 sprintf((char *)_sData.msg, "L:%f R:%f\n",Car_Motion.SpeedA,Car_Motion.SpeedB);
//			 xQueueSend(xMyQueueHandle, &_sData, 0);
//			
//			 sprintf((char *)_sData.msg, "LA:%d RA:%d\n", Lazer.Left_Average, Lazer.Right_Average);
			
            // 发送到队列
            if (xQueueSend(xMyQueueHandle, &_sData, 0) != pdPASS)
            {
                printf("Queue Full\n");  // 如果队列已满
            }

            d10 = 0;  // 重置计数器
        }
    }
}
void Sender_task(void *pvParameters)
{
   struct messageQueue r_queue;
	char temp[100];
	int j1=1;
	while(1)
	{
		//接收数据  如果数据接收成功就处理 否则就点亮LED4
		if( pdTRUE == xQueueReceive( xMyQueueHandle,&r_queue,portMAX_DELAY) )
		{
			printf("%s",r_queue.msg);
			printf("\n");
		}
//      printf("%d",Lazer.Front);
	  vTaskDelay(1);
	}
}


//void basic_task(void *pvParameters)
//{
//	static uint8_t d10;
//	while(1)
//	{
//		d10 ++;
//		vTaskDelay(100);
//		if(d10 >= 5)
//		{
//			LED_Turn();
////			Car_Motion.Accumulation_X_Flag = 1;
////			printf("F:%d",Lazer.Front);
//			printf(" LA:%d",Lazer.Left_Average);printf(" RA:%d\n",Lazer.Right_Average);
////			printf("L:%f",Car_Motion.AngleA);printf(" R:%f",Car_Motion.AngleB);
//			printf("\n");
////			printf("L:%f",Car_Motion.SpeedA);printf(" R:%f",Car_Motion.SpeedB);
////			printf("L:%f\n",Car_Motion.Vx);
////			printf("%f\n",Car_Motion.Accumulation_X);

////			printf("\nF:%d",Lazer.Front);
////			printf(" LF:%d",Lazer.Left_Front);
////			printf(" LB:%d",Lazer.Left_Back);
////			printf(" RF:%d",Lazer.Right_Front);
////			printf(" RB:%d",Lazer.Right_Back);
////			printf("\nYAW:%d",Euler.yaw);
////			
////			
////			printf("\nH=%d S=%d L=%d",hsl.h,hsl.s,hsl.l);
//			
//			d10 = 0;
//		}
//	}
//}


void tcs_task(void *pvParameters)
{
 	while(1)
	{
		TCS34725_GetRawData(&rgb,1);  
		RGBtoHSL(&rgb,&hsl);
	  //RGBto255RGB(&rgb,&rgb255);
	    vTaskDelay(40);
	}
}



void rcc_task(void *pvParameters)
{
	while(1)
	{
		
		status = PcdRequest(PICC_REQALL, g_ucTempbuf);//寻卡

		if (status != MI_OK)
		{    
			PcdReset();
			PcdAntennaOff(); 
			PcdAntennaOn(); 
			continue;
		}
		status = PcdAnticoll(g_ucTempbuf);
		if (status != MI_OK)
		{    
			continue;    
		}
//		for(i=0;i<4;i++)
//		{
//		temp=g_ucTempbuf[i];
//		printf("%X",temp);
//		}
		status = PcdSelect(g_ucTempbuf);
		if (status != MI_OK)
		{    
			continue;    
		}	
		    status != MI_OK;
        status = PcdRead(0x05, data2);//扇区，数据
//        if(status == MI_OK)//读卡成功
//        {
////            printf("DATA:");
////            printf("\r\n");
////						//uid序列号
////						for(i = 3; i < 16; i++)
////            {
////                printf("%02x", data2[i]);
////            }
////            printf("\r\n");
//        }
//        else
//        {
//            printf("PcdRead() failed\r\n");
//        }
				if((data2[3]==0x65)&&(data2[4]==0x6E)&&(data2[5]==0x31)&&(data2[6]==0x32))
				{
				sprintf((char *)_sData.msg, "position 12\n"); 
 				xQueueSend(xMyQueueHandle, &_sData, 0);
//				printf("\r\n");
				}
				
				if((data2[3]==0x65)&&(data2[4]==0x6E)&&(data2[5]==0x31)&&(data2[6]==0xFE))
				{
				sprintf((char *)_sData.msg, "position 1\n"); 
 				xQueueSend(xMyQueueHandle, &_sData, 0);
//				printf("\r\n");
				}
		
				
				
				PcdHalt(); 
		vTaskDelay(100);	
		PcdReset();
	}
}

void offset_Init(void)
{
	int16_t buffer;
	uint16_t times = 0;
//	LED2_ON();
	while(times<20)
	{
		uint8_t check = 1;
		buffer = Side_Range(1);
		if (buffer > 180)
		{
			check = 0;
		}
		else 
		{
			Lazer.Right_Front = filter1(buffer);
		}
		buffer = Side_Range(2);
		if (buffer > 180)
		{
			check = 0;
		}
		else  
		{
			Lazer.Right_Back = filter2(buffer);	
		}		
		buffer = Side_Range(3);
		if (buffer > 180)
		{
			check = 0;
		}
		else 
		{
			Lazer.Left_Front = filter3(buffer);
		}
		buffer = Side_Range(4);
		if (buffer > 180)
		{
			check = 0;
		}
		else 
		{
			Lazer.Left_Back = filter4(buffer);
		}		
		if(check)
		{
			times ++;
		}
	  delay_ms(100);//vTaskDelay(100);
	}
	
	Lazer.Left_Front_Offset = 75 - Lazer.Left_Front;
	Lazer.Left_Back_Offset = 75 - Lazer.Left_Back;
	Lazer.Right_Front_Offset = 75 - Lazer.Right_Front;
	Lazer.Right_Back_Offset = 75 - Lazer.Right_Back;
	delay_ms(100);//vTaskDelay(100);
//	LED2_OFF();
}

void Lazer_Average(void)
{
	if(Lazer.Right_Front >= 200)
	{
		if(Lazer.Right_Back >= 200)
		{
			Lazer.Right_Average = 250;
		}
		else
		{
			Lazer.Right_Average = Lazer.Right_Back;
		}
	}
	else if(Lazer.Right_Back >= 200)
	{
		Lazer.Right_Average = Lazer.Right_Front;
	}
	else
	{
		Lazer.Right_Average = (Lazer.Right_Front + Lazer.Right_Back) / 2;
	}
	
	if(Lazer.Left_Front >= 200)
	{
		if(Lazer.Left_Back >= 200)
		{
			Lazer.Left_Average = 250;
		}
		else
		{
			Lazer.Left_Average = Lazer.Left_Back;
		}
	}
	else if(Lazer.Left_Back >= 200)
	{
		Lazer.Left_Average = Lazer.Left_Front;
	}
	else
	{
		Lazer.Left_Average = (Lazer.Left_Front + Lazer.Left_Back) / 2;
	}
	
//	Lazer.Right_Average = Lazer.Right_Back;
//	Lazer.Left_Average = Lazer.Left_Back;
	
}
void Lazer_task(void *pvParameters)
{
	int16_t buffer;
	while(1)
	{
		buffer = Side_Range(1) + Lazer.Right_Front_Offset;
		if ((buffer > 180)||(buffer < 20))
			Lazer.Right_Front = 250;
		else 
			Lazer.Right_Front = filter1(buffer);
		vTaskDelay(5);	
		Lazer_Average();
		buffer = Side_Range(2) + Lazer.Right_Back_Offset;
		if ((buffer > 180)||(buffer < 20))
			Lazer.Right_Back = 250;
		else 
			Lazer.Right_Back = filter2(buffer);
		vTaskDelay(5);	
		Lazer_Average();
		buffer = Side_Range(3) + Lazer.Left_Front_Offset;
		if ((buffer > 180)||(buffer < 20))
			Lazer.Left_Front = 250;
		else 
			Lazer.Left_Front = filter3(buffer);
		vTaskDelay(5);	
		Lazer_Average();
		buffer = Side_Range(4) + Lazer.Left_Back_Offset;
		if ((buffer > 180)||(buffer < 20))
			Lazer.Left_Back = 250;
		else 
			Lazer.Left_Back = filter4(buffer);
		vTaskDelay(5);	
		Lazer_Average();
	}
}
static char* left(char *dest,char *src ,int n){
	char *p=dest;
	char *q=src;
	int len=strlen(src);
	
	if(n>len){
		n=len;
	}
	while(n--) *(p++)=*(q++);
	*(p++)='\0';
	return dest;
}
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		char * a;
		char * b;	
		char back[20];		
		LazerRX.Rx_buff = USART_ReceiveData(USART2);
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		if(LazerRX.Rx_Check == 1)//当发现包头
		{
			LazerRX.Rx_String[LazerRX.Rx_Flag++] = LazerRX.Rx_buff;  //接收字符
			if(LazerRX.Rx_Flag >= 45)
			{
				memset(LazerRX.Rx_String,0x00,sizeof(LazerRX.Rx_buff)); //清空接收字符串
				LazerRX.Rx_Flag = 0; //清空计数器
				LazerRX.Rx_Check = 0;
			}
			else if(LazerRX.Rx_String[LazerRX.Rx_Flag-1] == '\n')//当收到包尾
			{
				a = strstr(LazerRX.Rx_String,":");
				b = strstr(LazerRX.Rx_String,"mm,");
				left(back,a+1,b-a-1);
				Lazer.Front = atoi(back);
				memset(LazerRX.Rx_String,0x00,sizeof(LazerRX.Rx_buff)); //清空接收字符串
				LazerRX.Rx_Flag = 0; //清空计数器				
				LazerRX.Rx_Check = 0;
			}
		}
		else
		{
			if(LazerRX.Rx_buff == 'D')
			{
				LazerRX.Rx_String[LazerRX.Rx_Flag++] = LazerRX.Rx_buff;  //接收字符
				LazerRX.Rx_Check = 1;
			}
			else
			{
				LazerRX.Rx_Flag = 0;
			}
		}
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}
void UART4_IRQHandler(void)
{
	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{
		uint8_t i,val_counter = 0;
		Master.Rx_buff = USART_ReceiveData(UART4);
		if(Master.Rx_Check == 1)//当发现包头
		{
			Master.Rx_String[Master.Rx_Flag++] = Master.Rx_buff;  //逐字接收字符
			if(Master.Rx_Flag >= 30)
			{
				memset(Master.Rx_String,0x00,sizeof(Master.Rx_buff)); //清空接收字符串
				Master.Rx_Flag = 0; //清空计数器			
				Master.Rx_Check = 0;
			}
			else if(Master.Rx_String[Master.Rx_Flag-1] == '#')//当收到包尾
			{
				for(i=0;i<=40;i++)
				{
					Master.Rx_String_Copy[i] = Master.Rx_String[i];
					Master.Rx_String[i] = 0;
				}
				Master.RX_OK = 1;
				memset(Master.Rx_String,0x00,sizeof(Master.Rx_buff)); //清空接收字符串
				Master.Rx_Flag = 0; //清空计数器				
				Master.Rx_Check = 0;
			}
		}
		else
		{
			if(Master.Rx_buff == '@')
			{
				Master.Rx_String[Master.Rx_Flag++] = Master.Rx_buff;  //接收字符
				Master.Rx_Check = 1;
			}
			else
			{
				Master.Rx_Flag = 0;
			}
		}
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}
}
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t i,val_counter = 0;
		Master.Rx_buff = USART_ReceiveData(USART1);
		if(Master.Rx_Check == 1)//当发现包头
		{
			Master.Rx_String[Master.Rx_Flag++] = Master.Rx_buff;  //逐字接收字符
			if(Master.Rx_Flag >= 30)
			{
				memset(Master.Rx_String,0x00,sizeof(Master.Rx_buff)); //清空接收字符串
				Master.Rx_Flag = 0; //清空计数器			
				Master.Rx_Check = 0;
			}
			else if(Master.Rx_String[Master.Rx_Flag-1] == '#')//当收到包尾
			{
				for(i=0;i<=40;i++)
				{
					Master.Rx_String_Copy[i] = Master.Rx_String[i];
					Master.Rx_String[i] = 0;
				}
				Master.RX_OK = 1;
				memset(Master.Rx_String,0x00,sizeof(Master.Rx_buff)); //清空接收字符串
				Master.Rx_Flag = 0; //清空计数器				
				Master.Rx_Check = 0;
			}
		}
		else
		{
			if(Master.Rx_buff == '@')
			{
				Master.Rx_String[Master.Rx_Flag++] = Master.Rx_buff;  //接收字符
				Master.Rx_Check = 1;
			}
			else
			{
				Master.Rx_Flag = 0;
			}
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		MPU.Rx_buff = USART_ReceiveData(USART3);
		if(MPU.Rx_Check == 1)//当发现包头
		{			
			MPU.Rx_String[MPU.Rx_Flag++] = MPU.Rx_buff;
			if(MPU.Rx_Flag > 10)
			{
				if(MPU.Rx_String[1] == 0x53)//如果是角度
				{
					Euler.yaw = (MPU.Rx_String[7] << 8)+ MPU.Rx_String[6];
					if(Euler.yaw_offset_get == 0){Euler.yaw_offset = Euler.yaw;Euler.yaw_offset_get = 1;}
					
				}
				else if(MPU.Rx_String[1] == 0x52)//如果是角速度
				{
					Euler.GYO = (MPU.Rx_String[7] << 8)+ MPU.Rx_String[6];
				}
				memset(MPU.Rx_String,0x00,sizeof(MPU.Rx_buff)); //清空接收字符串
				MPU.Rx_Flag = 0; //清空计数器
				MPU.Rx_Check = 0;
			}
			else if(MPU.Rx_Flag >= 45)
			{
				memset(MPU.Rx_String,0x00,sizeof(MPU.Rx_buff)); //清空接收字符串
				MPU.Rx_Flag = 0; //清空计数器				
				MPU.Rx_Check = 0;
			}			
		}
		else
		{
			if(MPU.Rx_buff == 0x55)//判断包头
			{
				MPU.Rx_String[MPU.Rx_Flag++] = MPU.Rx_buff;  //接收字符
				MPU.Rx_Check = 1;
			}
			else
			{
				MPU.Rx_Flag = 0;
			}
		}
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}
}
void FOC_ANGLE_SET(uint8_t moter,float angle)
{
	uint8_t data[] = {5,0,0,0,0,0,0,0};//第二位1或2代表电机
	long temp;
	data[1] = moter;

	if(angle > 0)
	{
		data[2] = 1;
		temp = angle * 100;
	}
	else
	{
		temp = -angle * 100;
	}
	
	data[3] = (temp / 10000 % 100);
	data[4] = (temp / 100 % 100);
	data[5] = (temp / 1 % 100);
	
	if(Can_Send_Msg(data,8) == 0)
	{
	}
}
