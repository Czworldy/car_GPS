#ifndef __GPS_H
#define __GPS_H

#include "sys.h"
#include "MyTypeDef.h"

#define GPS_STATE_SIZE	2

extern struct GPS_State GPS_List[];

void GPS_Update(void);
void GPS_Clear(void);
void GPS_Init(double x, double y);

#endif
