#ifndef __LED_H
#define __LED_H

#include "sys.h"

#define LED_ON  0
#define LED_OFF 1

#define LED0 PHout(14)
#define LED1 PHout(15)
#define LED2 PIout(0)
#define LED3 PIout(1)

void LED_Init(void);	//≥ı ºªØ

#endif
