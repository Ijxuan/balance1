#ifndef FPS_Calculate_H
#define FPS_Calculate_H

#include "main.h"


typedef struct 
{
    uint32_t WorldTime;             //����ʱ��
    uint32_t Last_WorldTime;        //��һ��ʱ��
}WorldTime_RxTypedef;
//֡�ʼ���

typedef struct 
{
	//�Ӿ��Ľ�������֡��
	WorldTime_RxTypedef WorldTimes;
	uint32_t FPS;							//֡��
	
}FPS_type;

typedef struct 
{
	FPS_type Vision;
	FPS_type IMU_angle;
	FPS_type IMU_Gyro;
	FPS_type YAW_6020;
	FPS_type PITCH_6020;
	FPS_type CHASSIS_PLACE;
	FPS_type DEBUG;

	
}FPS_ALL_type;

extern FPS_ALL_type FPS_ALL;


//֡�ʼ���

//	//�Ӿ��Ľ�������֡��
//	WorldTime_RxTypedef Vision_WorldTimes;
//	uint32_t FPS;							//֡��


void Get_FPS(WorldTime_RxTypedef *time,uint32_t* FPS);



#endif















