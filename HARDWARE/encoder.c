#include "encoder.h"
#include "qei.h"
#include "gyro.h"
#include "gps.h"
#include "mymath.h"
#include "lcd.h"
#include "touch.h"
#include "delay.h"
#include "M_Func.h"
#include "usart.h"

struct Encoder_State Encoders[EncoderAmount]=
{
	{0, 0, 0, 1.0, 1.0, 0.0, 1.0,  0.0, 1},
	{0, 0, 0, 1.0, 1.0, 0.0, 1.0, pi/2, 1},
};

int32_t Encoder_N[EncoderAmount*2] = {0};//用来标定码盘时记录用的

int32_t Get_Encoder_Pulse_Count(u8 i)
{
	int32_t temp;
	temp = QEI_overflow_cnt[i] * (1 + QEI_TIM[i]->ARR) + QEI_TIM[i]->CNT;
	return temp;
}

void Encoder_Update(void)
{
	u8 i;
	int32_t now;
	int32_t step;
	
	for (i = 0; i < EncoderAmount; ++i)
	{
		now	= Get_Encoder_Pulse_Count(i);
		step = now - Encoders[i].Now;
		Encoders[i].Now = now;
		Encoders[i].Total = Encoders[i].Now - Encoders[i].Last;
		if (step >= 0)
			Encoders[i].Distance += step * Encoders[i].Convert1 * Encoders[i].dir;
		else
			Encoders[i].Distance += step * Encoders[i].Convert2 * Encoders[i].dir;
	}
}

void Encoder_Clear(u8 index)
{
	Encoders[index].Distance = 0.0;
	Encoders[index].Last = Encoders[index].Now;
	Encoders[index].Total = 0;
}

/****************************************************************************************************
码盘标定过程：分别沿2个方向推三米，记录两个码盘的读数，以下为N数组与码盘读数的对应关系
     0     90
E0  N[0]  N[1]
E1  N[2]  N[3]
****************************************************************************************************/
void Encoder_InitXY(u8 t)
{
	u8 i;
	for (i = 0; i < EncoderAmount; ++i)
		Encoder_N[i*2+t] = Encoders[i].Total;
}
/****************************************************************************************************
码盘旋转半径标定过程：
圈数  0     5
 E0	 N[0]  N[1]
 E1	 N[2]  N[3]
****************************************************************************************************/
void Encoder_InitR(void)
{
	u8 i;
	int32_t step;
	double dis;
	
	for (i = 0; i < EncoderAmount; ++i)
	{
		step = Encoder_N[i*2+1] - Encoder_N[i*2+0];
		if (step > 0)
			dis = step * Encoders[i].Convert1 * Encoders[i].dir;
		else
			dis = step * Encoders[i].Convert2 * Encoders[i].dir;
		Encoders[i].Radius = dis / (10 * pi + (Gyro_N[1] - Gyro_N[0]) * pi / 32768);
	}
}

/****************************************************************************************************
码盘标定过程：分别沿2个方向推三米，记录两个码盘的读数，以下为N数组与码盘读数的对应关系
     0     90
E0  N[0]  N[1]
E1  N[2]  N[3]
****************************************************************************************************/
void Encoder_Init(void)
{
	u8		i;
	double	C;					//转动系数
	double	angle;				//与+x夹角
	double	tmp;
	double	length = 3000.0;
	double	phi1 = -Gyro_N[0] * pi / 32768;	//第一次标定时车移动方向与+x夹角
	double	phi2 = -Gyro_N[1] * pi / 32768 + pi;	//第三次标定时车移动方向与+x夹角
	
	//近似计算公式
	for (i = 0; i < EncoderAmount; ++i)
	{
		angle = atan((Encoder_N[i*2+1]*cos(phi1)-Encoder_N[i*2]*cos(phi2))/(Encoder_N[i*2]*sin(phi2)-Encoder_N[i*2+1]*sin(phi1)));
		tmp = cos(angle-phi1);
		if (tmp < -0.1 || tmp > 0.1)
		{
			C = length*tmp/Encoder_N[i*2];
		}
		else
		{
			C = length*cos(angle-phi2)/Encoder_N[i*2+1];
		}
		Encoders[i].radian = angle;
		/* 正转、反转系数近似相等 */
		Encoders[i].Convert1 = Encoders[i].Convert2	= C;
	}
}

void Encoder_Init_Convert(void)
{
	u8 i, tp_last, key_value;
	
	KeyBoard_State = 0;
	Show_Keyboard();
	POINT_COLOR=BLACK;
	LCD_printf(0,6+36*0,300,24,24,"If sure, touch the screen");
	
	while (1)
	{
		#ifndef USING_TOUCH_SCREEN
		if (USART_Key_Receive)
		{
			USART_Key_Receive = 0;
			key_value = Key_RxBuffer[0];
		#else
		tp_last = tp_dev.sta&TP_PRES_DOWN;
		tp_dev.scan(0); 		 
		if ((tp_dev.sta&TP_PRES_DOWN) && !tp_last)
		{			//触摸屏被按下	
		 	if (tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{
				key_value = TP_Row_Judge(tp_dev.x[0], tp_dev.y[0]);
		#endif
				if (key_value == keyback)
					return;
				else if (key_value == keyboardtab)
				{
					KeyBoard_State = !KeyBoard_State;
					Show_Keyboard();
				}
				else
					break;
			#ifndef USING_TOUCH_SCREEN
			#else
			}
			#endif
		}
		else
			delay_ms(1);
	}
	
	KeyBoard_State = 0;
	Show_Keyboard();
	POINT_COLOR=BLACK;
	LCD_printf(0,6+36*0,300,24,24,"Along Angle 0:");
	LCD_printf(0,6+36*1,300,24,24,"Find a reference line");
	LCD_printf(0,6+36*2,300,24,24,"Then touch the screen to start");
	
	while (1)
	{
		#ifndef USING_TOUCH_SCREEN
		if (USART_Key_Receive)
		{
			USART_Key_Receive = 0;
			key_value = Key_RxBuffer[0];
		#else
		tp_last = tp_dev.sta&TP_PRES_DOWN;
		tp_dev.scan(0); 		 
		if ((tp_dev.sta&TP_PRES_DOWN) && !tp_last)
		{			//触摸屏被按下	
		 	if (tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{
				key_value = TP_Row_Judge(tp_dev.x[0], tp_dev.y[0]);
		#endif
				if (key_value == keyback)
					return;
				else if (key_value == keyboardtab)
				{
					KeyBoard_State = !KeyBoard_State;
					Show_Keyboard();
				}
				else
					break;
			#ifndef USING_TOUCH_SCREEN
			#else
			}
			#endif
		}
		else
			delay_ms(1);
	}
	
	for (i = 0; i < EncoderAmount; ++i)
		Encoder_Clear(i);
	Gyro_Set_Zero();
	delay_ms(1000);
	Gyro_N[0] = 0;//Gyro_Total;
	
	KeyBoard_State = 0;
	Show_Keyboard();
	POINT_COLOR=BLACK;
	LCD_printf(0,6+36*0,300,24,24,"Along Angle 0:Push 3m");
	LCD_printf(0,6+36*1,300,24,24,"Touch screen to end");
	
	while (1)
	{
		LCD_printf(0,6+36*2,300,24,24,"1.Angle = %lf",Gyro_Angle_Total);
		LCD_printf(0,6+36*3,300,24,24,"2.Total 1 = %lld",Encoders[0].Total);
		LCD_printf(0,6+36*4,300,24,24,"3.Total 2 = %lld",Encoders[1].Total);
		
		#ifndef USING_TOUCH_SCREEN
		if (USART_Key_Receive)
		{
			USART_Key_Receive = 0;
			key_value = Key_RxBuffer[0];
		#else
		tp_last = tp_dev.sta&TP_PRES_DOWN;
		tp_dev.scan(0); 		 
		if ((tp_dev.sta&TP_PRES_DOWN) && !tp_last)
		{			//触摸屏被按下	
		 	if (tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{
				key_value = TP_Row_Judge(tp_dev.x[0], tp_dev.y[0]);
		#endif
				if (key_value == keyback)
					return;
				else if (key_value == keyboardtab)
				{
					KeyBoard_State = !KeyBoard_State;
					Show_Keyboard();
				}
				else
					break;
			#ifndef USING_TOUCH_SCREEN
			#else
			}
			#endif
		}
		else
			delay_ms(1);
	}
	
	Encoder_InitXY(0);
	
	KeyBoard_State = 0;
	Show_Keyboard();
	POINT_COLOR=BLACK;
	LCD_printf(0,6+36*0,300,24,24,"Along Angle 90:");
	LCD_printf(0,6+36*1,300,24,24,"Find a reference line");
	LCD_printf(0,6+36*2,300,24,24,"Then touch the screen to start");
	
	while (1)
	{
		#ifndef USING_TOUCH_SCREEN
		if (USART_Key_Receive)
		{
			USART_Key_Receive = 0;
			key_value = Key_RxBuffer[0];
		#else
		tp_last = tp_dev.sta&TP_PRES_DOWN;
		tp_dev.scan(0); 		 
		if ((tp_dev.sta&TP_PRES_DOWN) && !tp_last)
		{			//触摸屏被按下	
		 	if (tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{
				key_value = TP_Row_Judge(tp_dev.x[0], tp_dev.y[0]);
		#endif
				if (key_value == keyback)
					return;
				else if (key_value == keyboardtab)
				{
					KeyBoard_State = !KeyBoard_State;
					Show_Keyboard();
				}
				else
					break;
			#ifndef USING_TOUCH_SCREEN
			#else
			}
			#endif
		}
		else
			delay_ms(1);
	}
	
	for (i = 0; i < EncoderAmount; ++i)
		Encoder_Clear(i);
	Gyro_N[1] = -(32768 / 2);//Gyro_Total;
	
	KeyBoard_State = 0;
	Show_Keyboard();
	POINT_COLOR=BLACK;
	LCD_printf(0,6+36*0,300,24,24,"Along Angle 90:Push 3m");
	LCD_printf(0,6+36*1,300,24,24,"Touch screen to end");
	
	while (1)
	{
		LCD_printf(0,6+36*2,300,24,24,"1.Angle = %lf",Gyro_Angle_Total);
		LCD_printf(0,6+36*3,300,24,24,"2.Total 1 = %lld",Encoders[0].Total);
		LCD_printf(0,6+36*4,300,24,24,"3.Total 2 = %lld",Encoders[1].Total);
		
		#ifndef USING_TOUCH_SCREEN
		if (USART_Key_Receive)
		{
			USART_Key_Receive = 0;
			key_value = Key_RxBuffer[0];
		#else
		tp_last = tp_dev.sta&TP_PRES_DOWN;
		tp_dev.scan(0); 		 
		if ((tp_dev.sta&TP_PRES_DOWN) && !tp_last)
		{			//触摸屏被按下	
		 	if (tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{
				key_value = TP_Row_Judge(tp_dev.x[0], tp_dev.y[0]);
		#endif
				if (key_value == keyback)
					return;
				else if (key_value == keyboardtab)
				{
					KeyBoard_State = !KeyBoard_State;
					Show_Keyboard();
				}
				else
					break;
			#ifndef USING_TOUCH_SCREEN
			#else
			}
			#endif
		}
		else
			delay_ms(1);
	}
	
	Encoder_InitXY(1);
	
	Encoder_Init();
	Encoder_View_Coefficient();
}

void Encoder_Init_Radius(void)
{
	u8 i, tp_last, key_value;
	u8 gyro_cnt = 0;
	u8 gyro_state = 0;
	
	LCD_Clear(WHITE);
	KeyBoard_State = 0;
	Show_Keyboard();
	POINT_COLOR=BLACK;
	LCD_printf(0,6+36*0,300,24,24,"If sure, touch the screen");
	
	while (1)
	{
		#ifndef USING_TOUCH_SCREEN
		if (USART_Key_Receive)
		{
			USART_Key_Receive = 0;
			key_value = Key_RxBuffer[0];
		#else
		tp_last = tp_dev.sta&TP_PRES_DOWN;
		tp_dev.scan(0); 		 
		if ((tp_dev.sta&TP_PRES_DOWN) && !tp_last)
		{			//触摸屏被按下	
		 	if (tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{
				key_value = TP_Row_Judge(tp_dev.x[0], tp_dev.y[0]);
		#endif
				if (key_value == keyback)
					return;
				else if (key_value == keyboardtab)
				{
					KeyBoard_State = !KeyBoard_State;
					Show_Keyboard();
				}
				else
					break;
			#ifndef USING_TOUCH_SCREEN
			#else
			}
			#endif
		}
		else
			delay_ms(1);
	}
	
	KeyBoard_State = 0;
	Show_Keyboard();
	POINT_COLOR=BLACK;
	LCD_printf(0,6+36*0,300,24,24,"Touch the screen to start");
	
	while (1)
	{
		#ifndef USING_TOUCH_SCREEN
		if (USART_Key_Receive)
		{
			USART_Key_Receive = 0;
			key_value = Key_RxBuffer[0];
		#else
		tp_last = tp_dev.sta&TP_PRES_DOWN;
		tp_dev.scan(0); 		 
		if ((tp_dev.sta&TP_PRES_DOWN) && !tp_last)
		{			//触摸屏被按下	
		 	if (tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{
				key_value = TP_Row_Judge(tp_dev.x[0], tp_dev.y[0]);
		#endif
				if (key_value == keyback)
					return;
				else if (key_value == keyboardtab)
				{
					KeyBoard_State = !KeyBoard_State;
					Show_Keyboard();
				}
				else
					break;
			#ifndef USING_TOUCH_SCREEN
			#else
			}
			#endif
		}
		else
			delay_ms(1);
	}
	
	for (i = 0; i < EncoderAmount; ++i)
		Encoder_Clear(i);
	Gyro_Set_Zero();
	delay_ms(1000);
	
	KeyBoard_State = 0;
	Show_Keyboard();
	POINT_COLOR=BLACK;
	LCD_printf(0,6+36*0,300,24,24,"1.Angle:%lf",Gyro_Angle_Total);
	LCD_printf(0,6+36*1,300,24,24,"2.E0 Dis:%lf",Encoders[0].Distance);
	LCD_printf(0,6+36*2,300,24,24,"3.E1 Dis:%lf",Encoders[1].Distance);
	
	while (Gyro_Angle_Total > -170);
	
	while (1)
	{
		if (Gyro_Angle_Total >= 10)
		{
			if (!gyro_state)
			{
				Gyro_N[0] = Gyro_Total;
				Encoder_InitXY(0);
				++gyro_cnt;
				gyro_state = 1;
				break;
			}
		}
	}
	
	while (1)
	{
		if (Gyro_Angle_Total >= 10)
		{
			if (!gyro_state)
			{
				++gyro_cnt;
				gyro_state = 1;
			}
		}
		else if (Gyro_Angle_Total <= -170)
		{
			if (gyro_state)
			{
				++gyro_cnt;
				gyro_state = 0;
			}
		}
		if (gyro_cnt > 10)
		{
			Gyro_N[1] = Gyro_Total;
			Encoder_InitXY(1);
			break;
		}
		LCD_printf(0,6+36*0,300,24,24,"1.Angle:%lf",Gyro_Angle_Total);
		LCD_printf(0,6+36*1,300,24,24,"2.E0 Dis:%lf",Encoders[0].Distance);
		LCD_printf(0,6+36*2,300,24,24,"3.E1 Dis:%lf",Encoders[1].Distance);
		LCD_printf(0,6+36*3,300,24,24,"3.Rotation CNT:%u",gyro_cnt);
		#ifndef USING_TOUCH_SCREEN
		if (USART_Key_Receive)
		{
			USART_Key_Receive = 0;
			key_value = Key_RxBuffer[0];
		#else
		tp_last = tp_dev.sta&TP_PRES_DOWN;
		tp_dev.scan(0); 		 
		if ((tp_dev.sta&TP_PRES_DOWN) && !tp_last)
		{			//触摸屏被按下	
		 	if (tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{
				key_value = TP_Row_Judge(tp_dev.x[0], tp_dev.y[0]);
		#endif
				if (key_value == keyback)
					return;
				else if (key_value == keyboardtab)
				{
					KeyBoard_State = !KeyBoard_State;
					Show_Keyboard();
				}
				else
					break;
			#ifndef USING_TOUCH_SCREEN
			#else
			}
			#endif
		}
	}
	
	Encoder_InitR();
	Encoder_View_Coefficient();
}

void Encoder_View_Coefficient(void)
{
	u8 tp_last, key_value;
	
	LCD_Clear(WHITE);
	KeyBoard_State = 0;
	Show_Keyboard();
	POINT_COLOR=BLACK;
	LCD_printf(0,6+36*0,320,24,24,"1.E0.C1:%.10lf",Encoders[0].Convert1);
	LCD_printf(0,6+36*1,320,24,24,"2.E0.C2:%.10lf",Encoders[0].Convert2);
	LCD_printf(0,6+36*2,320,24,24,"3.E0.R:%.10lf",Encoders[0].Radius);
	LCD_printf(0,6+36*3,320,24,24,"4.E0.rad:%.10lf",Encoders[0].radian);
	LCD_printf(0,6+36*4,320,24,24,"5.E1.C1:%.10lf",Encoders[1].Convert1);
	LCD_printf(0,6+36*5,320,24,24,"6.E1.C2:%.10lf",Encoders[1].Convert2);
	LCD_printf(0,6+36*6,320,24,24,"7.E1.R:%.10lf",Encoders[1].Radius);
	LCD_printf(0,6+36*7,320,24,24,"8.E1.rad:%.10lf",Encoders[1].radian);
	
	while (1)
	{
		#ifndef USING_TOUCH_SCREEN
		if (USART_Key_Receive)
		{
			USART_Key_Receive = 0;
			key_value = Key_RxBuffer[0];
		#else
		tp_last = tp_dev.sta&TP_PRES_DOWN;
		tp_dev.scan(0); 		 
		if ((tp_dev.sta&TP_PRES_DOWN) && !tp_last)
		{			//触摸屏被按下	
		 	if (tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{
				key_value = TP_Row_Judge(tp_dev.x[0], tp_dev.y[0]);
		#endif
				switch(key_value)
				{
					case keyback:
						return;
					case keyboardtab:
						KeyBoard_State = !KeyBoard_State;
						Show_Keyboard();
						break;
				}
				POINT_COLOR=BLACK;
				LCD_printf(0,6+36*0,320,24,24,"1.E0.C1:%.10lf",Encoders[0].Convert1);
				LCD_printf(0,6+36*1,320,24,24,"2.E0.C2:%.10lf",Encoders[0].Convert2);
				LCD_printf(0,6+36*2,320,24,24,"3.E0.R:%.10lf",Encoders[0].Radius);
				LCD_printf(0,6+36*3,320,24,24,"4.E0.rad:%.10lf",Encoders[0].radian);
				LCD_printf(0,6+36*4,320,24,24,"5.E1.C1:%.10lf",Encoders[1].Convert1);
				LCD_printf(0,6+36*5,320,24,24,"6.E1.C2:%.10lf",Encoders[1].Convert2);
				LCD_printf(0,6+36*6,320,24,24,"7.E1.R:%.10lf",Encoders[1].Radius);
				LCD_printf(0,6+36*7,320,24,24,"8.E1.rad:%.10lf",Encoders[1].radian);
			#ifndef USING_TOUCH_SCREEN
			#else
			}
			#endif
		}
		else
			delay_ms(1);
	}
}

void Encoder_View_Calibration(void)
{
	u8 i, tp_last, key_value;
	
	LCD_Clear(WHITE);
	KeyBoard_State = 0;
	Show_Keyboard();
	POINT_COLOR=BLACK;
	for (i = 0; i < 4; ++i)
		LCD_printf(0,6+36*i,300,24,24,"%u.EN[%u] = %lld",i+1,i,Encoder_N[i]);
	for (i = 0; i < 2; ++i)
		LCD_printf(0,6+36*(i+4),300,24,24,"%u.GN[%u] = %lld",i+5,i,Gyro_N[i]);
	
	while (1)
	{
		#ifndef USING_TOUCH_SCREEN
		if (USART_Key_Receive)
		{
			USART_Key_Receive = 0;
			key_value = Key_RxBuffer[0];
		#else
		tp_last = tp_dev.sta&TP_PRES_DOWN;
		tp_dev.scan(0); 		 
		if ((tp_dev.sta&TP_PRES_DOWN) && !tp_last)
		{			//触摸屏被按下	
		 	if (tp_dev.x[0]<lcddev.width&&tp_dev.y[0]<lcddev.height)
			{
				key_value = TP_Row_Judge(tp_dev.x[0], tp_dev.y[0]);
		#endif
				switch(key_value)
				{
					case keyback:
						return;
					case keyboardtab:
						KeyBoard_State = !KeyBoard_State;
						Show_Keyboard();
						break;
				}
				POINT_COLOR=BLACK;
				for (i = 0; i < 4; ++i)
					LCD_printf(0,6+36*i,300,24,24,"%u.EN[%u] = %lld",i+1,i,Encoder_N[i]);
				for (i = 0; i < 2; ++i)
					LCD_printf(0,6+36*(i+4),300,24,24,"%u.GN[%u] = %lld",i+5,i,Gyro_N[i]);
			#ifndef USING_TOUCH_SCREEN
			#else
			}
			#endif
		}
		else
			delay_ms(1);
	}
}
