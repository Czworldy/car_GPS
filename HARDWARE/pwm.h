#ifndef __PWM_H
#define __PWM_H

#include "sys.h"

#define PWM1_FWD PHout(5)
#define PWM1_REV PHout(4)
#define PWM2_FWD PAout(4)
#define PWM2_REV PAout(5)
#define PWM3_FWD PAout(7)
#define PWM3_REV PAout(6)
#define PWM4_FWD PCout(4)
#define PWM4_REV PCout(5)

typedef struct
{
	u32 clk_freq;
	u32 pwm_freq;
	u8  tim_num;
	u8  chx;
	u8  gpiox;
	u8  pinx;
	double duty_init;
}PWM_t;

extern PWM_t pwm_structs[];

void PWM_Dir_Init(void);
void PWM_Init(void);
double PWM_GetDuty(u8 pwmx);
u8 PWM_SetDuty(u8 pwmx, float duty);

#endif
