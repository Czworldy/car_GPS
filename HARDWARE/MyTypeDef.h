/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYTYPEDEF_H
#define __MYTYPEDEF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//点结构体
struct Point
{
	double x;
	double y;
};

struct Encoder_State
{
	int32_t Now;
	int32_t Last;
	int32_t Total;
	double Convert1;		//正转系数
	double Convert2;		//反转系数
	double Distance;
	double Radius;			//车身自转时码盘的旋转半径
	double radian;			//码盘与车身坐标系X轴正方向的夹角
	int8_t dir;				//改变正反转方向（如果是需要的方向则置1，否则置-1）
};

struct GPS_State
{
	struct Point position;	//当前坐标	
	double distancestep1; 	//码盘1本次前进距离
	double distancestep2;   //码盘2本次前进距离
	double distance1;	  	//码盘1前进距离
	double distance2;		//码盘2前进距离
	double angle;        	//当前角度 		
	double radian;			//当前弧度
};

//按键值
enum M_KeyNumType
{
    key1 = 1,
    key2,
    key3,
    key4,
    keyback,//5
    emergency,//6
    
    key5,//7
    key6,//8
    key7,//9
    key8,//10
    keyempty1,//11
    keyempty2,//12
    
    key9,//13
    key0,
    keypoint,
    keysign,
    keydelete,
    keyOK,
	keyboardtab
};
#define pageup point
#define pagedown sign

#endif 

/**********************************END OF FILE*********************************/
