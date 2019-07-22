#include "all.h"

int main(void)
{
	u8 tp_last, key_value;
	u32 all_command;
	double temp_x, temp_y;
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	LED_Init();
	
	LED0 = LED1 = LED2 = LED3 = LED_ON;
	
	Encoders[0].Convert1	= 0.0199694759; //0.1922755959;//-0.0199632611;//0.0199632611;//0.0199894676;//0.0199118659;//0.0200207215;//-0.02002;//-0.02001;//-0.0199009638570108;//-0.0199309092179547;//-0.0199407066149716;//-0.0199049682982943;//0.0199982219658630;//0.0200560870879295;//0.197310138146330;//0.197007863072000;//0.198840280992025;
	Encoders[0].Convert2	= 0.0199694759; //0.1922755959;//-0.0199632611;//0.0199632611;//0.0199894676;//0.0199118659;//0.0200207215;//-0.02002;//-0.02001;//-0.0199196563367001;//-0.0199499630714902;//-0.0199189189910357;//-0.0199085752482390;//0.0200352132205636;//0.0201891795919431;//0.197406521454356;//0.197169730335748;//0.195967189027243;
	Encoders[0].Radius		= -90.2811635872;//19.26196024;//11.0313375814;//-11.0313375814;//-7.5194874761;//-8.5898182664;//-202;//-172.817487719820;//-174.598988075254;//-176.631210968137;//-194.098816136950;//159.574530502035;//127.020017989383;//185.42531;//190.1599;//190.12818;//191.4008;
	Encoders[0].radian		= -1.5667337222; //1.571601512;//0.0008051850+pi/2;//0.0008051850;//0.0006811259;//0.0050635560;//0.0028104915;//0.77777;//0.77785;//0.77832;//-0.799939641908259;//-0.798119134781297;//-0.796995023558119;//-0.788640567077689;//-0.793959391850915;//0.757099851466271;//-0.769667673054887;//0.767369899231028;//0.758607287901989;	//-0.77629;	//-0.81406;
	Encoders[1].Convert1	= -0.0199746379; //-0.0201123415;//-0.0199147441;//0.0199147441;//0.0199193833;//0.0200039442;//0.0199546354507972;//-0.01993;//-0.02;//-0.0199546354507972;//0.0199456254196184;//0.0199987792928170;//0.0199343488110157;//0.0200208461777850;//0.0196794392011745;//0.198422387040686;//0.198200920803355;//0.198679448611520;
	Encoders[1].Convert2	= -0.0199746379;//-0.0201123415;//-0.0199147441;//0.0199147441;//0.0199193833;//0.0200039442;//0.0199546354507972;//-0.01993;//-0.02;//-0.0199606537022622;//0.0199855526033765;//0.0200018174534947;//0.0199681587861943;//0.0199861907573634;//0.0196685244399883;//0.198215368901519;//0.198315800946308;//0.199680347126157;
	Encoders[1].Radius		= -19.5199932296; //82.11124767415;//-78.2351078344;//78.2351078344;//80.9321044890;//83.3386897826;//-202;//132.363962206486;//131.172655578694;//131.849608808693;//110.743711056953;//162.965989286186;//131.478481342326;//187.88986;//184.50149;//183.75073;//184.0632;
	Encoders[1].radian		= 0.0030738468; //0.0007344148;//test-1.570061912+pi/2;//-1.570061912;//1.5706160408;//1.5546657803;//1.5706806944;//-0.77285;//-0.77351;//-0.776889080180455;//0.780181577214815;//0.779231418756681;//0.796427268666153;//0.790605246653594;//-0.777988601502375;//0.770188222396010;//0.766854295238844;//0.762407628892167;
	
	GPS_TxBuffer[0] = 0xAB;
	GPS_TxBuffer[1] = 0xBC;
	GPS_TxBuffer[2] = 0xCD;
	GPS_TxBuffer[3] = 0xDE;
	GPS_TxBuffer[GPS_TxBufferSize-4] = 0xED;
	GPS_TxBuffer[GPS_TxBufferSize-3] = 0xDC;
	GPS_TxBuffer[GPS_TxBufferSize-2] = 0xCB;
	GPS_TxBuffer[GPS_TxBufferSize-1] = 0xBA;
	
	QEI_Init();
	PWM_Init();
	USART_Config();
	Gyro_USART_DMA_EN();
	GPS_Clear();
	
#ifdef BSP_USING_LCD
	LCD_Init();           //初始化LCD FSMC接口
	LCD_Scan_Dir(R2L_D2U);
	
	use_touch_adj_data = 1;
	tp_dev.init();				//触摸屏初始化
	
	LCD_Clear(WHITE);
	POINT_COLOR=BLACK;
	LCD_printf(0,6+36*0,300,24,24,"1.Angle = %lf        ",GPS_List[0].angle);
	LCD_printf(0,6+36*1,300,24,24,"2.X = %lf        ",GPS_List[0].position.x);
	LCD_printf(0,6+36*2,300,24,24,"3.Y = %lf        ",GPS_List[0].position.y);
	LCD_printf(0,6+36*3,300,24,24,"4.E0 dis = %lf        ",Encoders[0].Distance);
	LCD_printf(0,6+36*4,300,24,24,"5.E1 dis = %lf        ",Encoders[1].Distance);
	LCD_printf(0,6+36*5,300,24,24,"6.Gyro Set Zero");
	LCD_printf(0,6+36*6,300,24,24,"7.Encoder 0 Clear");
	LCD_printf(0,6+36*7,300,24,24,"8.Encoder 1 Clear");
	LCD_printf(0,6+36*8,300,24,24,"9.GPS Clear");
	LCD_printf(0,6+36*9,300,24,24,"10.Duty:%.2f,%.2f,%.2f,%.2f     ",PWM_GetDuty(1),PWM_GetDuty(2),PWM_GetDuty(3),PWM_GetDuty(4));
#endif
	
	LED0 = LED1 = LED2 = LED3 = LED_OFF;
	
	while(1)
	{
		if (Is_GPS_Command)
		{
		#ifdef BSP_USING_LCD
			LCD_printf(0,6+36*10,300,24,24,"%02X %02X %02X  ",Command_Index[0],Command_Index[1],Command_Index[2]);
			LCD_printf(0,6+36*11,300,24,24,"%02X %02X %02X %02X  %f      ",Command_Context[0],Command_Context[1],Command_Context[2],Command_Context[3],(*((float*)Command_Context + 0)));
		#endif
			all_command = (Command_Index[2] << 16 | Command_Index[1] << 8 | Command_Index[0]);
			switch (all_command)
			{
				case 0xAA00AA:
					GPS_Clear();
					break;
				case 0xAA01AA:
					temp_x = (*((float*)Command_Context + 0));
					GPS_Init(temp_x,GPS_List[0].position.y);
					break;
				case 0xAA02AA:
					temp_y = (*((float*)Command_Context + 0));
					GPS_Init(GPS_List[0].position.x, temp_y);
					break;
				case 0xAA03AA:
					Gyro_Set_Zero();
					break;
				case 0xAB00AB:
					PWM_SetDuty(1,0);
					PWM_SetDuty(2,0);
					PWM_SetDuty(3,0);
					PWM_SetDuty(4,0);
					break;
				case 0xAB01AB:
					PWM_SetDuty(1,(float)Command_Context[0]);
					PWM_SetDuty(2,(float)Command_Context[1]);
					PWM_SetDuty(3,(float)Command_Context[2]);
					PWM_SetDuty(4,(float)Command_Context[3]);
					break;
				case 0xAC00AC:
					Encoder_Init_Convert();
					break;
				case 0xAC01AC:
					Encoder_Init_Radius();
					break;
				default:
					break;
			}
			Is_GPS_Command = 0;
		}
	#ifdef BSP_USING_LCD
		LCD_printf(0,6+36*0,300,24,24,"1.Angle = %lf        ",Gyro_Angle_Total);//GPS_List[0].angle);
		LCD_printf(0,6+36*1,300,24,24,"2.X = %lf        ",GPS_List[0].position.x);
		LCD_printf(0,6+36*2,300,24,24,"3.Y = %lf        ",GPS_List[0].position.y);
		LCD_printf(0,6+36*3,300,24,24,"4.E0 dis = %lf        ",Encoders[0].Distance);
		LCD_printf(0,6+36*4,300,24,24,"5.E1 dis = %lf        ",Encoders[1].Distance);
		LCD_printf(0,6+36*9,300,24,24,"10.Duty:%.2f,%.2f,%.2f,%.2f     ",PWM_GetDuty(1),PWM_GetDuty(2),PWM_GetDuty(3),PWM_GetDuty(4));
		tp_last = tp_dev.sta&TP_PRES_DOWN;
		tp_dev.scan(0);
		if ((tp_dev.sta&TP_PRES_DOWN) && !tp_last)
		{			//触摸屏被按下	
			if (tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{
				key_value = TP_Row_Judge(tp_dev.x[0], tp_dev.y[0]);
				switch(key_value)
				{
					case key6:
						Gyro_Set_Zero();
						break;
					case key7:
						Encoder_Clear(0);
						break;
					case key8:
						Encoder_Clear(1);
						break;
					case key9:
						GPS_Clear();
						break;
					default:
						break;
				}
				POINT_COLOR=BLACK;
				LCD_printf(0,6+36*0,300,24,24,"1.Angle = %lf        ",GPS_List[0].angle);
				LCD_printf(0,6+36*1,300,24,24,"2.X = %lf        ",GPS_List[0].position.x);
				LCD_printf(0,6+36*2,300,24,24,"3.Y = %lf        ",GPS_List[0].position.y);
				LCD_printf(0,6+36*3,300,24,24,"4.E0 dis = %lf        ",Encoders[0].Distance);
				LCD_printf(0,6+36*4,300,24,24,"5.E1 dis = %lf        ",Encoders[1].Distance);
				LCD_printf(0,6+36*5,300,24,24,"6.Gyro Set Zero");
				LCD_printf(0,6+36*6,300,24,24,"7.Encoder 0 Clear");
				LCD_printf(0,6+36*7,300,24,24,"8.Encoder 1 Clear");
				LCD_printf(0,6+36*8,300,24,24,"9.GPS Clear");
				LCD_printf(0,6+36*9,300,24,24,"10.Duty:%.2f,%.2f,%.2f,%.2f     ",PWM_GetDuty(1),PWM_GetDuty(2),PWM_GetDuty(3),PWM_GetDuty(4));
			}
		}
		else
			delay_ms(1);
	#endif
	}
}
