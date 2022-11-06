#ifndef FPS_Calculate_H
#define FPS_Calculate_H

#include "main.h"


typedef struct 
{
    uint32_t WorldTime;             //世界时钟
    uint32_t Last_WorldTime;        //上一刻时钟
}WorldTime_RxTypedef;
//帧率计算

typedef struct 
{
	//视觉的接收数据帧率
	WorldTime_RxTypedef WorldTimes;
	uint32_t FPS;							//帧率
	
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


//帧率计算

//	//视觉的接收数据帧率
//	WorldTime_RxTypedef Vision_WorldTimes;
//	uint32_t FPS;							//帧率


void Get_FPS(WorldTime_RxTypedef *time,uint32_t* FPS);



#endif















