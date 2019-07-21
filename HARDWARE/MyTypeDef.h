/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYTYPEDEF_H
#define __MYTYPEDEF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//��ṹ��
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
	double Convert1;		//��תϵ��
	double Convert2;		//��תϵ��
	double Distance;
	double Radius;			//������תʱ���̵���ת�뾶
	double radian;			//�����복������ϵX��������ļн�
	int8_t dir;				//�ı�����ת�����������Ҫ�ķ�������1��������-1��
};

struct GPS_State
{
	struct Point position;	//��ǰ����	
	double distancestep1; 	//����1����ǰ������
	double distancestep2;   //����2����ǰ������
	double distance1;	  	//����1ǰ������
	double distance2;		//����2ǰ������
	double angle;        	//��ǰ�Ƕ� 		
	double radian;			//��ǰ����
};

//����ֵ
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
