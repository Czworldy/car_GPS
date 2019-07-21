#ifndef __QEI_H
#define __QEI_H

#include "sys.h"

extern volatile int32_t QEI_overflow_cnt[];
extern TIM_TypeDef* QEI_TIM[];

void QEI_Init(void);

#endif
