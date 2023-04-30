#ifndef __CAN2_SEND_RM_H
#define __CAN2_SEND_RM_H

#include "main.h"
//#include "BSP_CAN.h"		//二选一
#include "user_can.h"		//二选一

#include "RM_JudgeSystem.h"

#define SPEED_CHANGE_SEND_ID 0x145



#define PLACE_SEND_ID 0x144


#define JS_SEND_HEAT_ID_ONE 0x142
#define JS_SEND_HEAT_ID_TWO 0x143 

#define JS_SEND_STATUS_ID_ONE 0x138
#define JS_SEND_STATUS_ID_TWO 0x139
#define JS_SEND_STATUS_ID_THREE 0x140
#define JS_SEND_STATUS_ID_FOUR 0x141

#define JS_SEND_HURT_ID 0x137

#define JS_SEND_SHOOT_ID 0x136


void JS_send_SHOOT_control(void);
void JS_send_HURT_control(void);
void JS_send_STATUS_control(void);
void JS_send_HEAT_control(void);
void PLACE_send_control(void);
void SPEED_CHANGE_SEND_control(void);

#endif



