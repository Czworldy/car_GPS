#include "gyro.h"
#include "usart.h"
#include "gps.h"

int16_t Gyro_Total;
double Gyro_Angle_Total;
double Gyro_Radian_Total;

int16_t Gyro_N[2] = {0};

void Gyro_Set_Zero(void)
{
	u8 i;
#ifdef BSP_USING_USART6
	DMA_Cmd(USART6_DMA_RX_STREAM, DISABLE);
	while (DMA_GetCmdStatus(USART6_DMA_RX_STREAM) != DISABLE) {} 
	
	USART_SendByte(USART6, 0xFF);
	USART_SendByte(USART6, 0xAA);
	USART_SendByte(USART6, 0x69);
	USART_SendByte(USART6, 0x88);
	USART_SendByte(USART6, 0xb5);
		
	USART_SendByte(USART6, 0xFF);
	USART_SendByte(USART6, 0xAA);
	USART_SendByte(USART6, 0x76);
	USART_SendByte(USART6, 0x00);
	USART_SendByte(USART6, 0x00);
		
#endif
	
	Gyro_Total = 0;
	Gyro_Angle_Total = Gyro_Radian_Total = 0.0;
	for (i = 0; i < GPS_STATE_SIZE; ++i)
	{
		GPS_List[i].angle = 0.0;
		GPS_List[i].radian = 0.0;
	}
	
	Gyro_USART_DMA_EN();
}
