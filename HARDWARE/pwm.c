#include "pwm.h"

typedef enum
{PA, PB, PC, PD, PE, PF, PG, PH, PI}eGPIO_t;

PWM_t pwm_structs[] = 
{
//   clk_freq      pwm_freq        timx    	   chx         gpiox       pinx       duty_init
	{168000,		200,			9,			1,			PE,			5,           0},//49},
	{168000,		200,			9,			2,			PE,			6,           0},//23},
	{168000,		200,			10,			1,			PF,			6,           0},//30},
	{168000,		200,			11,			1,			PF,			7,           0},//15},
};//端口按次序分别为PWM1,PWM2,PWM3,PWM4

static const u8 pwm_num = sizeof(pwm_structs) / sizeof(pwm_structs[0]);

static u8 GetAPBx(u8 timx)
{
	if(timx > 14) 
		return 0xff;
	else if((timx >= 2 && timx <= 7) || (timx >= 12 && timx <= 14))
		return 1;
	else
		return 2;
}

static u8 Get_AF(u8 timx)
{
	if(timx <= 2)
		return (u8)1;
	else if(timx <= 5)
		return (u8)2;
	else if(timx <= 11)
		return (u8)3;
	else if(timx <= 14)
		return (u8)9;
	else
		return 0xff;
}

static u32 Get_TimRCC(u8 timx)
{
	if(timx <= 1)
		return (u32)1;
	else if(timx <= 7)
		return (u32)(1 << (timx - 2));
	else if(timx <= 8)
		return (u32)2;
	else if(timx <= 11)
		return (u32)(1 << (timx + 7));
	else if(timx <= 14)
		return (u32)(1 << (timx - 6));
	else
		return 0xff;
}

static void PWM_GPIO_Init(PWM_t *pwm_struct)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_TypeDef *gpiox = (GPIO_TypeDef *)(AHB1PERIPH_BASE + 0x0400 * pwm_struct->gpiox);
	u16 pin_bit = 1 << pwm_struct->pinx;
	
	RCC_AHB1PeriphClockCmd((u32)(1 << pwm_struct->gpiox), ENABLE);
	GPIO_InitStructure.GPIO_Pin = pin_bit;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(gpiox, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(gpiox, (u8)pwm_struct->pinx, Get_AF(pwm_struct->tim_num));
}

static TIM_TypeDef *Get_TIMx(u8 timx)
{
	switch (timx)
	{
		case 1:  return TIM1;
		case 2:  return TIM2;
		case 3:  return TIM3;
		case 4:  return TIM4;
		case 5:  return TIM5;
		case 6:  return TIM6;
		case 7:  return TIM7;
		case 8:  return TIM8;
		case 9:  return TIM9;
		case 10: return TIM10;
		case 11: return TIM11;
		case 12: return TIM12;
		case 13: return TIM13;
		case 14: return TIM14;
		default: break;
	}
	return (TIM_TypeDef *)0;
}

/**
  * @brief  PWM波的时钟初始化
  * @note   
  * @param  None
  * @retval None
  */
static void PWM_Clk_Init(PWM_t *pwm_struct)
{   
	u16 presc;
	u16 cycle;
	u32 p_freq;
	u8  APBx;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	APBx = GetAPBx(pwm_struct->tim_num);
	p_freq = (APBx == 1) ? (SystemCoreClock/2) : SystemCoreClock;
	presc  = p_freq / pwm_struct->clk_freq;
	cycle  = pwm_struct->clk_freq / pwm_struct->pwm_freq;
	if(APBx == 1)
		RCC_APB1PeriphClockCmd(Get_TimRCC(pwm_struct->tim_num), ENABLE); 
	else
		RCC_APB2PeriphClockCmd(Get_TimRCC(pwm_struct->tim_num), ENABLE); 
	
	TIM_TimeBaseStructure.TIM_Period = cycle - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = presc - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(Get_TIMx(pwm_struct->tim_num), &TIM_TimeBaseStructure);
}

/**
  * @brief  PWM波的输出配置
  * @note   
  * @param  None
  * @retval None
  */
static void PWM_OCInit(PWM_t *pwm_struct)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TypeDef *timx = Get_TIMx(pwm_struct->tim_num);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = pwm_struct->duty_init * pwm_struct->clk_freq / pwm_struct->pwm_freq / 100;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	switch (pwm_struct->chx)
	{
		case 1:
			TIM_OC1Init(timx, &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(timx, TIM_OCPreload_Enable);
			break;
		case 2:
			TIM_OC2Init(timx, &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(timx, TIM_OCPreload_Enable);
			break;
		case 3:
			TIM_OC3Init(timx, &TIM_OCInitStructure);
			TIM_OC3PreloadConfig(timx, TIM_OCPreload_Enable);            
			break;
		case 4:
			TIM_OC4Init(timx, &TIM_OCInitStructure);
			TIM_OC4PreloadConfig(timx, TIM_OCPreload_Enable);            
			break;
		default:
			break;
	}
	TIM_Cmd(timx, ENABLE);
}

void PWM_Dir_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOH, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	PWM1_FWD = PWM1_REV = PWM2_FWD = PWM2_REV = PWM3_FWD = PWM3_REV = PWM4_FWD = PWM4_REV = 0;
}

/**
  * @brief  PWM波的初始化
  * @note   
  * @param  None
  * @retval None
  */
void PWM_Init(void)
{
	int i;
	
	PWM_Dir_Init();
	
	for(i = 0; i < pwm_num; ++i)
	{
		PWM_GPIO_Init(&pwm_structs[i]);
		PWM_Clk_Init(&pwm_structs[i]);
		PWM_OCInit(&pwm_structs[i]);
	}
}

/**
  * @brief  得到某路周期
  * @note   
  * @param  None
  * @retval None
  */
static u32 Get_Cycle(u8 pwmx)
{
	assert_param(pwmx >= 1 && pwmx <= 4);
	return pwm_structs[pwmx-1].clk_freq /pwm_structs[pwmx-1].pwm_freq;
}

/**
  * @brief  设置某一路PWM波的占空比
  * @note   
  * @param  pwmx:第几路pwm波
            duty:要设置成占空比多少
                 20, 即是20%的占空比
  * @retval 1
  */
u8 PWM_SetDuty(u8 pwmx, float duty)
{
	TIM_TypeDef * timx = Get_TIMx(pwm_structs[pwmx-1].tim_num);
	u32 ccrx = duty * Get_Cycle(pwmx) / 100;
	
	assert_param(pwmx >= 1 && pwmx <= pwm_num);
	assert_param(duty <= 100);
	switch (pwm_structs[pwmx-1].chx)
	{
		case 1: timx->CCR1 = ccrx; break;
		case 2: timx->CCR2 = ccrx; break;
		case 3: timx->CCR3 = ccrx; break;
		case 4: timx->CCR4 = ccrx; break;
		default: break;
	}
	return 1;
}

/**
  * @brief  得到PWM波的占空比
  * @note   
  * @param  PWMx:第几路占空比,范围根据PWM使用了几路的情况，最大为8
                 从1开始计数，即没有第0路
  * @retval 返回0，即调用错误
  */
double PWM_GetDuty(u8 pwmx)
{
	TIM_TypeDef * timx = Get_TIMx(pwm_structs[pwmx-1].tim_num);
	u32 ccrx;
	u32 cycle = pwm_structs[pwmx-1].clk_freq / pwm_structs[pwmx-1].pwm_freq;
	
	assert_param(pwmx >= 1 && pwmx <= pwm_num);
	
	switch(pwm_structs[pwmx-1].chx)
	{
		case 1: ccrx = timx->CCR1; break;
		case 2: ccrx = timx->CCR2; break;
		case 3: ccrx = timx->CCR3; break;
		case 4: ccrx = timx->CCR4; break;
		default: break;
	}
	
	return ccrx*100.0/cycle;
}
