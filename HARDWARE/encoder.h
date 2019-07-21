#ifndef __ENCODER_H
#define __ENCODER_H

#include "sys.h"
#include "MyTypeDef.h"

#define EncoderAmount 2

extern struct Encoder_State Encoders[];
extern int32_t Encoder_N[];

int32_t Get_Encoder_Pulse_Count(u8 i);
void Encoder_Update(void);
void Encoder_Clear(u8 index);
void Encoder_Init(void);
void Encoder_InitXY(u8 t);
void Encoder_InitR(void);

void Encoder_Init_Convert(void);
void Encoder_Init_Radius(void);
void Encoder_View_Coefficient(void);
void Encoder_View_Calibration(void);

#endif
