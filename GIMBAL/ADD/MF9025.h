#ifndef Mf9025_H
#define Mf9025_H

#include "main.h"
//#include "BSP_CAN.h"
#include "user_can.h"

#define MF9025_READID_START 0x141 //CAN接收到 MF9025的电调 开始ID
#define MF9025_Broadcast_Mode_id 0x280 //广播模式控制标识符



typedef struct
{
	uint8_t command;	 //读回来的电机温度
	uint16_t real_encoder_Angle;	 //读回来的机械角度
	uint16_t last_angle; //读回来的机械角度
	int16_t realSpeed;	 //读回来的速度
	int16_t real_torque_Current; //读回来的实际转矩电流
	uint8_t temperature;	 //读回来的电机温度
	
	float OutputCurrent; //M3508输出电流
	uint16_t targetAngle; //目标角度
	float targetSpeed;  //目标速度
	int16_t turnCount;	//转过的圈数
	float totalAngle; //转过的总角度
	uint16_t OffLine_Detection; //离线检测
	uint8_t OffLine_Status;		//离线标志位

} MF9025_t;
extern  MF9025_t MF9025[2];
extern  int send_to_MF9025_TEST;
extern int mf9025_tgg_speed;


void MF9025_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);

void MF9025_setTorque(int16_t Iid1,int16_t Iid2,int16_t Iid3,int16_t Iid4);








#endif
