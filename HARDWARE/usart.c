#include "usart.h"
#include "gyro.h"
#include "encoder.h"
#include "gps.h"
#include "mymath.h"
#include "command.h"

__align(32) volatile u8 Command_RxBuffer[Command_RxBufferSize];
__align(32) volatile u8 GPS_TxBuffer[GPS_TxBufferSize];
__align(32) volatile u8 Gyro_RxBuffer[Gyro_RxBufferSize];

void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

#ifdef BSP_USING_USART1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(USART1_RX_GPIO_CLK | USART1_TX_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(USART1_RX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;
	GPIO_Init(USART1_TX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(USART1_RX_GPIO_PORT, USART1_RX_SOURCE, GPIO_AF_USART1);
	GPIO_PinAFConfig(USART1_TX_GPIO_PORT, USART1_TX_SOURCE, GPIO_AF_USART1);
	
	USART_InitStructure.USART_BaudRate = USART1_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
#ifndef BSP_USING_USART1_DMA
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ClearFlag(USART1, USART_FLAG_RXNE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	
	USART_Cmd(USART1, ENABLE);
#else
	RCC_AHB1PeriphClockCmd(USART1_DMA_CLK, ENABLE);
	
	DMA_InitStructure.DMA_Channel = USART1_DMA_RX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (USART1_BASE+0x04);
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Handle_RxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = Handle_RxBufferSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART1_DMA_RX_STREAM, &DMA_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_DMA_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ClearFlag(USART1, USART_FLAG_RXNE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	DMA_ClearITPendingBit(USART1_DMA_RX_STREAM, USART1_DMA_RX_IT_TC);
	DMA_ITConfig(USART1_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
	
	DMA_Cmd(USART1_DMA_RX_STREAM, DISABLE);
	while (DMA_GetCmdStatus(USART1_DMA_RX_STREAM) != DISABLE) {}
	
	USART_Cmd(USART1, ENABLE);
#endif
#endif

#ifdef BSP_USING_USART2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(USART2_RX_GPIO_CLK | USART2_TX_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = USART2_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = USART2_TX_PIN;
	GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(USART2_RX_GPIO_PORT, USART2_RX_SOURCE, GPIO_AF_USART2);
	GPIO_PinAFConfig(USART2_TX_GPIO_PORT, USART2_TX_SOURCE, GPIO_AF_USART2);
	
	USART_InitStructure.USART_BaudRate = USART2_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	
#ifndef BSP_USING_USART2_DMA
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ClearFlag(USART2, USART_FLAG_RXNE);
	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	
	USART_Cmd(USART2, ENABLE);
#else
	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_Cmd(USART2, ENABLE);
	USART_SendByte(USART2, 0xA5);
	USART_SendByte(USART2, 0x81);
	USART_SendByte(USART2, 0x26);
	USART_Cmd(USART2, DISABLE);
	delay_ms(10);
	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_ClearFlag(USART2, USART_FLAG_RXNE);
	
	RCC_AHB1PeriphClockCmd(USART2_DMA_CLK, ENABLE);
	
	DMA_InitStructure.DMA_Channel = USART2_DMA_RX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (USART2_BASE+0x04);
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Color_RxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = Color_RxBufferSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART2_DMA_RX_STREAM, &DMA_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_DMA_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ClearFlag(USART2, USART_FLAG_RXNE);
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	DMA_ClearITPendingBit(USART2_DMA_RX_STREAM, USART2_DMA_RX_IT_TC);
	DMA_ITConfig(USART2_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
	
	DMA_Cmd(USART2_DMA_RX_STREAM, DISABLE);
	while (DMA_GetCmdStatus(USART2_DMA_RX_STREAM) != DISABLE) {}
	
	USART_Cmd(USART2, ENABLE);
#endif
#endif

#ifdef BSP_USING_USART3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(USART3_RX_GPIO_CLK | USART3_TX_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
	GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(USART3_RX_GPIO_PORT, USART3_RX_SOURCE, GPIO_AF_USART3);
	GPIO_PinAFConfig(USART3_TX_GPIO_PORT, USART3_TX_SOURCE, GPIO_AF_USART3);
	
	USART_InitStructure.USART_BaudRate = USART3_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

#ifndef BSP_USING_USART3_DMA
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ClearFlag(USART3, USART_FLAG_RXNE);
	USART_ClearFlag(USART3, USART_FLAG_TC);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	
	USART_Cmd(USART3, ENABLE);
#else
	RCC_AHB1PeriphClockCmd(USART3_DMA_CLK, ENABLE);
	
	DMA_InitStructure.DMA_Channel = USART3_DMA_RX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (USART3_BASE+0x04);
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Command_RxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = Command_RxBufferSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART3_DMA_RX_STREAM, &DMA_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_DMA_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	DMA_InitStructure.DMA_Channel = USART3_DMA_TX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (USART3_BASE+0x04);
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)GPS_TxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = GPS_TxBufferSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART3_DMA_TX_STREAM, &DMA_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_DMA_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ClearFlag(USART3, USART_FLAG_RXNE);
	USART_ClearFlag(USART3, USART_FLAG_TC);
	
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	DMA_ClearITPendingBit(USART3_DMA_RX_STREAM, USART3_DMA_RX_IT_TC);
	DMA_ClearITPendingBit(USART3_DMA_TX_STREAM, USART3_DMA_TX_IT_TC);
	DMA_ITConfig(USART3_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
	DMA_ITConfig(USART3_DMA_TX_STREAM, DMA_IT_TC, ENABLE);
	
	DMA_Cmd(USART3_DMA_RX_STREAM, ENABLE);
	DMA_Cmd(USART3_DMA_TX_STREAM, DISABLE);
	
	USART_Cmd(USART3, ENABLE);
#endif
#endif

#ifdef BSP_USING_UART4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_AHB1PeriphClockCmd(UART4_RX_GPIO_CLK | UART4_TX_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = UART4_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UART4_RX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = UART4_TX_PIN;
	GPIO_Init(UART4_TX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(UART4_RX_GPIO_PORT, UART4_RX_SOURCE, GPIO_AF_UART4);
	GPIO_PinAFConfig(UART4_TX_GPIO_PORT, UART4_TX_SOURCE, GPIO_AF_UART4);
	
	USART_InitStructure.USART_BaudRate = UART4_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ClearFlag(UART4, USART_FLAG_RXNE);
	USART_ClearFlag(UART4, USART_FLAG_TC);
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART4, ENABLE);
#endif

#ifdef BSP_USING_UART5
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_AHB1PeriphClockCmd(UART5_RX_GPIO_CLK | UART5_TX_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = UART5_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UART5_RX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = UART5_TX_PIN;
	GPIO_Init(UART5_TX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(UART5_RX_GPIO_PORT, UART5_RX_SOURCE, GPIO_AF_UART5);
	GPIO_PinAFConfig(UART5_TX_GPIO_PORT, UART5_TX_SOURCE, GPIO_AF_UART5);
	
	USART_InitStructure.USART_BaudRate = UART5_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ClearFlag(UART5, USART_FLAG_RXNE);
	USART_ClearFlag(UART5, USART_FLAG_TC);
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	USART_Cmd(UART5, ENABLE);
#endif

#ifdef BSP_USING_USART6
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	RCC_AHB1PeriphClockCmd(USART6_RX_GPIO_CLK | USART6_TX_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = USART6_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(USART6_RX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = USART6_TX_PIN;
	GPIO_Init(USART6_TX_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(USART6_RX_GPIO_PORT, USART6_RX_SOURCE, GPIO_AF_USART6);
	GPIO_PinAFConfig(USART6_TX_GPIO_PORT, USART6_TX_SOURCE, GPIO_AF_USART6);
	
	USART_InitStructure.USART_BaudRate = USART6_BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART6, &USART_InitStructure);
	
#ifndef BSP_USING_USART6_DMA
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ClearFlag(USART6, USART_FLAG_RXNE);
	USART_ClearFlag(USART6, USART_FLAG_TC);
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);	
	USART_Cmd(USART6, ENABLE);
#else
	RCC_AHB1PeriphClockCmd(USART6_DMA_CLK, ENABLE);
	
	DMA_InitStructure.DMA_Channel = USART6_DMA_RX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (USART6_BASE+0x04);
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Gyro_RxBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = Gyro_RxBufferSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(USART6_DMA_RX_STREAM, &DMA_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART6_DMA_RX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_ClearFlag(USART6, USART_FLAG_RXNE);
	USART_ClearFlag(USART6, USART_FLAG_TC);
	
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	DMA_ClearITPendingBit(USART6_DMA_RX_STREAM, USART6_DMA_RX_IT_TC);
	DMA_ITConfig(USART6_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
	
	DMA_Cmd(USART6_DMA_RX_STREAM, DISABLE);
	while (DMA_GetCmdStatus(USART6_DMA_RX_STREAM) != DISABLE) {}
	
	USART_Cmd(USART6, ENABLE);
#endif
#endif
}

void Gyro_USART_DMA_EN(void)
{
#ifdef BSP_USING_USART6_DMA
	uint8_t gyro_start, j;
	uint8_t gyro_temp[Gyro_RxBufferSize] = {0};
	
	gyro_start = 0;
	j = 0;
	/*while (!gyro_start)
	
	{
		if (j == 0)
		{
			gyro_temp[j] = USART_GetByte(USART6);
			if (gyro_temp[j] == 0x55)
			{
				j = 1;
				gyro_temp[j] = USART_GetByte(USART6);
				if (gyro_temp[j] == 0x52)
					j = 2;
				else
					j = 0;
			}
			else
				j = 0;
		}
		else
		{
			gyro_temp[j++] = USART_GetByte(USART6);
			switch (j)
			{
				case Gyro_RxBufferSize / 2:
					if (((gyro_temp[0] + gyro_temp[1] + gyro_temp[2] + gyro_temp[3] + gyro_temp[4] + gyro_temp[5] + 
						gyro_temp[6] + gyro_temp[7] + gyro_temp[8] + gyro_temp[9]) & 0xff) != gyro_temp[10])
						j = 0;
					break;
				case (Gyro_RxBufferSize / 2 + 1):
					if (gyro_temp[11] != 0x55)
						j = 0;
					break;
				case (Gyro_RxBufferSize / 2 + 2):
					if (gyro_temp[12] != 0x53)
						j = 0;
					break;
				case Gyro_RxBufferSize:
					if (((gyro_temp[11] + gyro_temp[12] + gyro_temp[13] + gyro_temp[14] + gyro_temp[15] + gyro_temp[16] + 
						gyro_temp[17] + gyro_temp[18] + gyro_temp[19] + gyro_temp[20]) & 0xff) != gyro_temp[21])
						j = 0;
					else
					{
						gyro_start = 1;
						while (1)
						{
							if (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == SET)
							{
								break;
							}
						}
						DMA_Cmd(USART6_DMA_RX_STREAM, ENABLE);
					}
					break;
				default:
					break;
			}
		}
	}*/
		while (!gyro_start)
	{
		if (j == 0 || j == 1)
		{
			gyro_temp[j] = USART_GetByte(USART6);
			if (gyro_temp[j] == 0x55)
			{
				j = 1;
				gyro_temp[j] = USART_GetByte(USART6);
				if (gyro_temp[j] == 0x53)
				{
					j = 2;
				}
				else
				{
					j = 0;
				}
			}
			else
			{
				j = 0;
			}
		}
		else
		{
			gyro_temp[j++] = USART_GetByte(USART6);
			switch (j)
			{
				case 11:
					if (((gyro_temp[0] + gyro_temp[1] + gyro_temp[2] + gyro_temp[3] + gyro_temp[4] + gyro_temp[5] + 
						gyro_temp[6] + gyro_temp[7] + gyro_temp[8] + gyro_temp[9]) & 0xff) == gyro_temp[10])
					{
						gyro_start = 1;
						while (1)
						{
							if (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == SET)
							{
								break;
							}
						}
						DMA_Cmd(USART6_DMA_RX_STREAM, ENABLE);
					}
					else
					{
						j = 0;
					}
					break;
				default:
					break;
			}
		}
	}
#endif
}

void USART_SendByte(USART_TypeDef *USARTx, uint8_t dat)
{
	USART_SendData(USARTx, dat);
	while (!USART_GetFlagStatus(USARTx, USART_FLAG_TXE));
}

uint8_t USART_GetByte(USART_TypeDef *USARTx)
{
	uint8_t dat;
	while (!USART_GetFlagStatus(USARTx, USART_FLAG_RXNE));
	dat = (USART_ReceiveData(USARTx) & 0xff);
	return dat;
}

void USART_puts(USART_TypeDef * USARTx, char * str)
{
	while (*str)
	{
		USART_SendByte(USARTx, *str++);
	}
}

void USART_printf(USART_TypeDef * USARTx, char *fmt, ...)
{
	va_list ap;
	char str[128];

	va_start(ap,fmt);
	vsprintf(str,fmt,ap);
	va_end(ap);
	USART_puts(USARTx, str);
}

//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while ((Print_USART->SR&0X40)==0);//循环发送,直到发送完毕   
	Print_USART->DR = (u8) ch;      
	return ch;
}

#ifdef BSP_USING_USART1
#ifdef BSP_USING_USART1_DMA
void USART1_DMA_RX_IRQHandler(void)
{
	if (DMA_GetITStatus(USART1_DMA_RX_STREAM, USART1_DMA_RX_IT_TC) == SET)
	{
		if (((Handle_RxBuffer[3] << 8) | Handle_RxBuffer[2]))
			Handle_Key_Value = ((Handle_RxBuffer[3] << 8) | Handle_RxBuffer[2]);
		DMA_ClearITPendingBit(USART1_DMA_RX_STREAM, USART1_DMA_RX_IT_TC);
	}
}
#else
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		Key_RxBuffer[0] = USART1->DR;
		USART_Key_Receive = 1;
	}
}
#endif
#endif

#ifdef BSP_USING_USART2
#ifdef BSP_USING_USART2_DMA
void USART2_DMA_RX_IRQHandler(void)
{
	if (DMA_GetITStatus(USART2_DMA_RX_STREAM, USART2_DMA_RX_IT_TC) == SET)
	{
		Cur_Color.r = Color_RxBuffer[4];
		Cur_Color.g = Color_RxBuffer[5];
		Cur_Color.b = Color_RxBuffer[6];
		Cur_Color.rgb565 = ((((u16)Cur_Color.r >> 3) << 11) | (((u16)Cur_Color.g >> 2) << 5) | ((u16)Cur_Color.b >> 3));
		
		DMA_ClearITPendingBit(USART2_DMA_RX_STREAM, USART2_DMA_RX_IT_TC);
	}
}
#else
void USART2_IRQHandler(void)
{
	u8 c;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		c = USART2->DR;
		USART_printf(USART2, "%c", c);
	}
}
#endif
#endif

#ifdef BSP_USING_USART3
#ifdef BSP_USING_USART3_DMA
void USART3_DMA_RX_IRQHandler(void)
{
	if (DMA_GetITStatus(USART3_DMA_RX_STREAM, USART3_DMA_RX_IT_TC) == SET)
	{
		if (Command_RxBuffer[0] == 0xFF && Command_RxBuffer[Command_RxBufferSize-1] == 0x00)
		{
			Is_GPS_Command = 1;
			Command_Index[0] = Command_RxBuffer[1];
			Command_Index[1] = Command_RxBuffer[2];
			Command_Index[2] = Command_RxBuffer[3];
			Command_Context[0] = Command_RxBuffer[4];
			Command_Context[1] = Command_RxBuffer[5];
			Command_Context[2] = Command_RxBuffer[6];
			Command_Context[3] = Command_RxBuffer[7];
		}
		DMA_ClearITPendingBit(USART3_DMA_RX_STREAM, USART3_DMA_RX_IT_TC);
	}
}

void USART3_DMA_TX_IRQHandler(void)
{
	if (DMA_GetITStatus(USART3_DMA_TX_STREAM, USART3_DMA_TX_IT_TC) == SET)
	{
		DMA_Cmd(USART3_DMA_TX_STREAM, DISABLE);
		DMA_ClearITPendingBit(USART3_DMA_TX_STREAM, USART3_DMA_TX_IT_TC);
	}
}
#else
void USART3_IRQHandler(void)
{
	u8 c;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		c = USART3->DR;
		USART_printf(USART3, "%c", c);
	}
}
#endif
#endif

#ifdef BSP_USING_UART4
void UART4_IRQHandler(void)
{
	u8 c;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{ 	
		c = UART4->DR;
		USART_printf(UART4, "%c", c);
	} 
}
#endif

#ifdef BSP_USING_UART5
void UART5_IRQHandler(void)
{
	u8 c;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{ 	
		c = UART5->DR;
		USART_printf(UART5, "%c", c);
	} 
}
#endif

#ifdef BSP_USING_USART6
#ifdef BSP_USING_USART6_DMA
void USART6_DMA_RX_IRQHandler(void)
{
	if (DMA_GetITStatus(USART6_DMA_RX_STREAM, USART6_DMA_RX_IT_TC) == SET)
	{
		//Gyro_Total = ((Gyro_RxBuffer[18] << 8) | Gyro_RxBuffer[17]);
		Gyro_Total = ((Gyro_RxBuffer[7] << 8) | Gyro_RxBuffer[6]);
		Gyro_Angle_Total = Gyro_Total / 32768.0f*180.0f;
//		Gyro_Radian_Total = Gyro_Angle_Total * pi / 180.0f ;
		Gyro_Radian_Total = Gyro_Total * pi / 32768;
		
		Encoder_Update();
		
		GPS_Update();
		
		*((float*)GPS_TxBuffer + 1) = (float)(GPS_List[0].position.x);
		*((float*)GPS_TxBuffer + 2) = (float)(GPS_List[0].position.y);
		*((float*)GPS_TxBuffer + 3) = (float)(GPS_List[0].angle);
		
		DMA_SetCurrDataCounter(USART3_DMA_TX_STREAM, GPS_TxBufferSize);
		DMA_Cmd(USART3_DMA_TX_STREAM, ENABLE);
		
		DMA_ClearITPendingBit(USART6_DMA_RX_STREAM, USART6_DMA_RX_IT_TC);
	}
}
#else
void USART6_IRQHandler(void)
{
	u8 c;
	if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
	{
		c = USART6->DR;
		USART_printf(USART6, "%c", c);
	} 
}
#endif
#endif
