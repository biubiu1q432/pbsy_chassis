//void Mission_handler(void *pvParameters)
//{
////10ms���һ����������������
//	static uint16_t SPEEDUP_COUNTER = 0;
//	while(1)
//	{
//		SPEEDUP_COUNTER ++;
//		if(Mission.num == 1)
//		{
//			int16_t Front_DIS;
//			int16_t LEFT_DIS;
//			//ֱ�ж���
//			//parameter1����������ٶ�
//			//parameter2��ǰ��ֹͣ����
//			//parameter3�������������
//			Front_DIS = Lazer.Front - 50;//Mission.parameter2;
//			if((Euler.err <100) && (Euler.err >-100))
//			{
//				LEFT_DIS = Lazer.Side_Average - 50;
//				
//				if(((LEFT_DIS < -20) || (LEFT_DIS > 20)) && Front_DIS >= 100)//���ƫ����δ����Ŀ��(������������)
//				{
//					SPEEDUP_COUNTER = 0;
//					Car_Motion.VxTarget = 0;
//					while(Car_Motion.VxOrder >0.01){;}
//					
//					if(LEFT_DIS < 0){Euler.adjust = 3500;}//��ƫ
//					else{Euler.adjust = -3500;}//��ƫ
//					while((Euler.adjusterr >100) || (Euler.adjusterr <-100)){;}
//						
//					Car_Motion.VxTarget = 0.03;
//					vTaskDelay(2000);
//					Car_Motion.VxTarget = 0;
//					while(Car_Motion.VxTarget >0.01){;}
//						
//					Euler.adjust = 0;
//					while((Euler.adjusterr >100) || (Euler.adjusterr <-100)){;}
//				}
//				else
//				{
//					if(Front_DIS <= 0)
//					{
//						Car_Motion.VxTarget = -0.03;
//					}
//					else
//					{
//						if(Front_DIS <= 10)
//						{
//							Car_Motion.VxTarget = 0;
//							Mission.state = 0;
//							Mission.num = 0;
//						}
//						else if(Front_DIS <= 70)
//						{
//							Car_Motion.VxTarget = 0.03;
//						}
//						else if(Front_DIS <= 170)
//						{
//							Car_Motion.VxTarget = 0.05;
//						}


//						else
//						{
//							Car_Motion.VxTarget = 0.15;
////							if(Car_Motion.Vx <= 0.15)
////							{
////								if(SPEEDUP_COUNTER >= 10)
////								{
////									Car_Motion.VxTarget += 0.01;
////									SPEEDUP_COUNTER = 0;
////								}
////								
////							}

//						}						
//					}
//				}
//			}
//		}
//		else if(Mission.num == 2)
//		{
//			
//			
//			
//		}
//		
//		
//		
//		
//		vTaskDelay(10);
//	}
//}
