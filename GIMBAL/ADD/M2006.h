#ifndef M2006_H
#define M2006_H

#include "main.h"
//#include "BSP_CAN.h"
#include "user_can.h"


#define M2006_SENDID 	   0x200  //CAN���ͽo M3508�ĵ�� ID
#define M2006_READID_START 0x201 //CAN���յ� M3508�ĵ�� ��ʼID

extern	uint8_t M2006_RxMessageData[8];				//�洢CAN�������� ����


void M2006_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);




#endif
