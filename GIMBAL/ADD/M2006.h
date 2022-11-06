#ifndef M2006_H
#define M2006_H

#include "main.h"
//#include "BSP_CAN.h"
#include "user_can.h"


#define M2006_SENDID 	   0x200  //CAN发送給 M3508的电调 ID
#define M2006_READID_START 0x201 //CAN接收到 M3508的电调 开始ID

extern	uint8_t M2006_RxMessageData[8];				//存储CAN接收数据 数组


void M2006_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);




#endif
