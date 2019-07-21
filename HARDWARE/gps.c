#include "gps.h"
#include "gyro.h"
#include "encoder.h"
#include "mymath.h"

struct GPS_State GPS_List[GPS_STATE_SIZE] = {0};

inline void GPS_Update(void)
{
	double	radianstep;						//弧度步进值
	double	theta0b;
	double	theta1b;
	
	GPS_List[1] = GPS_List[0];
	GPS_List[0].angle = Gyro_Angle_Total;
	GPS_List[0].radian = Gyro_Radian_Total;
	
	radianstep = GPS_List[0].radian - GPS_List[1].radian;
	while (radianstep > pi)
		radianstep -= 2 * pi;
	while (radianstep < (-pi))
		radianstep += 2 * pi;
	
 	GPS_List[0].distance1 = Encoders[0].Distance;
 	GPS_List[0].distance2 = Encoders[1].Distance;
	
	GPS_List[0].distancestep1 = GPS_List[0].distance1 - GPS_List[1].distance1;
	GPS_List[0].distancestep2 = GPS_List[0].distance2 - GPS_List[1].distance2;
	
	GPS_List[0].distancestep1 -= Encoders[0].Radius*radianstep;
	GPS_List[0].distancestep2 -= Encoders[1].Radius*radianstep;
	theta0b = GPS_List[0].radian + Encoders[0].radian;
	theta1b = GPS_List[0].radian + Encoders[1].radian;
	GPS_List[0].position.x += ( sin(theta1b)*GPS_List[0].distancestep1 - sin(theta0b)*GPS_List[0].distancestep2) / sin(Encoders[1].radian-Encoders[0].radian);
	GPS_List[0].position.y += (-cos(theta1b)*GPS_List[0].distancestep1 + cos(theta0b)*GPS_List[0].distancestep2) / sin(Encoders[1].radian-Encoders[0].radian);
}

void GPS_Clear(void)
{
	GPS_Init(0,0);
	Gyro_Set_Zero();
}

void GPS_Init(double x, double y)
{
	u8 i;
	
	for (i = 0; i < EncoderAmount; ++i)
		Encoder_Clear(i);
	
	for (i = 0; i < GPS_STATE_SIZE; ++i)
	{
		GPS_List[i].distance1 = 0.0;
		GPS_List[i].distance2 = 0.0;
		GPS_List[i].distancestep1 = 0.0;
		GPS_List[i].distancestep2 = 0.0;
		GPS_List[i].position.x = x;
		GPS_List[i].position.y = y;
	}
}
