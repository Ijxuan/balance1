#ifndef DJI_IMU_H
#define DJI_IMU_H

#include "main.h"
#include "user_can.h"
#pragma anon_unions

#define send_way 0

//发送ID
#define DR16_Angle_SENDID 0x101

#define PLACE_SEND_ID 0x144
#define ENCODER_ID 0x152

#define DJI_C_Angle_SENDID 0x195
#define DJI_C_Gyro_SENDID 0x165
//陀螺仪校准 接收ID 
#define IMU_CAL_REIID 0x096

#define JS_SEND_HGAME_STATE_ID_ONE 0x150
#define JS_SEND_HGAME_STATE_ID_TWO 0x151

#define JS_SEND_HP_ID_ONE 0x146
#define JS_SEND_HP_ID_TWO 0x147
#define JS_SEND_HP_ID_THREE 0x148
#define JS_SEND_HP_ID_FOUR 0x149


#define JS_SEND_STATUS_ID_ONE 0x138
#define JS_SEND_STATUS_ID_TWO 0x139
#define JS_SEND_STATUS_ID_THREE 0x140
#define JS_SEND_STATUS_ID_FOUR 0x141

#define JS_RC_SHOOT_ID 0x136
#define JS_RC_HURT_ID 0x137

#define JS_SEND_HEAT_ID_ONE 0x142
#define JS_SEND_HEAT_ID_TWO 0x143 


#define DR16_R_PART_ONE 0x135
#define DR16_R_PART_TWO 0x134
#define DR16_R_PART_THREE 0x133


#define begin_calibration 1
#define stop_calibration 0
//以联合体形式发送，可以将float 类型的数据 转成 以uint8_t类型去发送
typedef union
{
	struct
	{
		int16_t DR16_CH0;
		int16_t DR16_CH1;
		int16_t DR16_CH2;
		int16_t DR16_CH3;
	};
	uint8_t DR16_SEND[8];
}DR16_Send_u;
extern DR16_Send_u DR16_T;

//以联合体形式发送，可以将float 类型的数据 转成 以uint8_t类型去发送
//欧拉角
typedef union
{
	struct
	{
		float yaw;
		float pitch;
	};
	uint8_t Euler_Angle[8];
}Euler_Send_u;
extern Euler_Send_u Euler_Send;

//角速度
typedef union
{
	struct
	{
		float Gyro_z;
		float Gyro_y;
	};
	uint8_t Gyro_zy[8];
}Gyro_Send_u;
extern Gyro_Send_u Gyro_Send;

//接收变量
//开始校准
typedef struct
{
	uint8_t real_Status;
	uint8_t last_Status;
}IMU_CAL_t;
extern IMU_CAL_t IMU_CAL;

#if send_way == 0
void DR16_Send_Fun(DR16_Send_u DR16_Send);


void Euler_Send_Fun(Euler_Send_u Euler_Send);
void Gyro_Send_Fun(Gyro_Send_u Gyro_Send);
void IMU_Cal_Status_Reivece(CAN_Rx_TypeDef CAN_Rx_Structure);
#endif
#if send_way == 1
void Euler_Send_Fun(float Yaw,float Pitch);
void Gyro_Send_Fun(float Gyro_z,float Gyro_y);
#endif


#endif

