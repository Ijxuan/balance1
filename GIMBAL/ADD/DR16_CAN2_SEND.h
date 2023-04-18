#ifndef __DR16_SEND_H
#define __DR16_SEND_H
#include "main.h"
#include "user_can.h"		//二选一
//#include "BSP_CAN.h"		//二选一


#define YAW_6020_SEND_ID 0x132


#define DR16_SEND_PART_ONE 0x135
#define DR16_SEND_PART_TWO 0x134
#define DR16_SEND_PART_THREE 0x133


#define send_shen_recive 0
//只在接收了之后才发送,即DR16离线则不发送
#define send_any_time 1
//任何时候都会以固定频率发送

//以联合体形式发送，可以将float 类型的数据 转成 以uint8_t类型去发送
//typedef union
//{
//	struct
//	{
//		int16_t DR16_CH0;
//		int16_t DR16_CH1;
//		int16_t DR16_CH2;
//		int16_t DR16_CH3;
//	};
//	uint8_t DR16_SEND[8];
//}DR16_Send_u;
//extern DR16_Send_u DR16_T;


#if send_any_time==1
//任何时候都会以固定频率发送
void DR16_send_master_control();//总控制

void DR16_send_part_one(void);
void DR16_send_part_two(void);
void DR16_send_part_three(void);

#endif



#endif




