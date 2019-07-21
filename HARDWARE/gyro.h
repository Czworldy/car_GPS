#ifndef __GYRO_H
#define __GYRO_H

#include "sys.h"

extern int16_t Gyro_Total;
extern double Gyro_Angle_Total;
extern double Gyro_Radian_Total;

extern int16_t Gyro_N[];

void Gyro_Set_Zero(void);

#endif
