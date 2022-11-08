#include "M3508.h"

M3508_t M3508s[4];

void M3508_setCurrent(int16_t Iid1,int16_t Iid2,int16_t Iid3,int16_t Iid4)
{
	uint8_t data[8];
	
	
	data[0] = Iid1 >> 8;				
	data[1] = Iid1;							
	data[2] = Iid2 >> 8;
	data[3] = Iid2;
	data[4] = Iid3 >> 8;
	data[5] = Iid3;
	data[6] = Iid4 >> 8;
	data[7] = Iid4;
	
	CAN_SendData(&hcan2,CAN_ID_STD,M3508_SENDID,data);
	
}

void M3508_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	
//	if(CAN_Rx_Structure.CAN_RxMessage.StdId != M3508_READID_START+3)
//		return;
	
	uint32_t EMID;				
	EMID = (uint32_t)(CAN_Rx_Structure.CAN_RxMessage.StdId - M3508_READID_START);				
	
	M3508s[EMID].realAngle = (uint16_t)(CAN_Rx_Structure.CAN_RxMessageData[0] << 8 | CAN_Rx_Structure.CAN_RxMessageData[1]);
	M3508s[EMID].realSpeed = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[2] << 8 | CAN_Rx_Structure.CAN_RxMessageData[3]);
	M3508s[EMID].realCurrent = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[4] << 8 | CAN_Rx_Structure.CAN_RxMessageData[5]);
	M3508s[EMID].temperture = CAN_Rx_Structure.CAN_RxMessageData[6];

		if(M3508s[EMID].realAngle - M3508s[EMID].last_angle > 4096)
		M3508s[EMID].turnCount --;
	else if (M3508s[EMID].realAngle - M3508s[EMID].last_angle < -4096)
		M3508s[EMID].turnCount ++;
	M3508s[EMID].totalAngle = M3508s[EMID].turnCount * 8192 + M3508s[EMID].realAngle;
	 M3508s[EMID].last_angle=M3508s[EMID].realAngle;
//	M3508s[EMID].OffLine_Detection++;	

}


/* ---------------------------------------------------------------------------------------------------- */

//M3508s1_t M3508s1[4];

void M3508s1_setCurrent(int16_t Iid1,int16_t Iid2,int16_t Iid3,int16_t Iid4)
{
	uint8_t data[8];
	
	
	data[0] = Iid1 >> 8;				
	data[1] = Iid1;							
	data[2] = Iid2 >> 8;
	data[3] = Iid2;
	data[4] = Iid3 >> 8;
	data[5] = Iid3;
	data[6] = Iid4 >> 8;
	data[7] = Iid4;
	
	CAN_SendData(&hcan1,CAN_ID_STD,M3508s1_SENDID,data);
	
}


void M3508s1_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{

//	if(CAN_Rx_Structure.CAN_RxMessage.StdId < M3508s1_START || CAN_Rx_Structure.CAN_RxMessage.StdId > M3508s1_END)
//		return;
	
	uint32_t EMID;				
	EMID = (uint32_t)(CAN_Rx_Structure.CAN_RxMessage.StdId - M3508s1_START);		

	M3508s[EMID].realAngle = (uint16_t)(CAN_Rx_Structure.CAN_RxMessageData[0] << 8 | CAN_Rx_Structure.CAN_RxMessageData[1]);
	M3508s[EMID].realSpeed = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[2] << 8 | CAN_Rx_Structure.CAN_RxMessageData[3]);
	M3508s[EMID].realCurrent = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[4] << 8 | CAN_Rx_Structure.CAN_RxMessageData[5]);
	M3508s[EMID].temperture = CAN_Rx_Structure.CAN_RxMessageData[6];
	
	
//	M3508s1[EMID].realAngle = (uint16_t)(CAN_Rx_Structure.CAN_RxMessageData[0] << 8 | CAN_Rx_Structure.CAN_RxMessageData[1]);
//	M3508s1[EMID].realSpeed = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[2] << 8 | CAN_Rx_Structure.CAN_RxMessageData[3]);
//	M3508s1[EMID].realCurrent = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[4] << 8 | CAN_Rx_Structure.CAN_RxMessageData[5]);
//	M3508s1[EMID].temperture = CAN_Rx_Structure.CAN_RxMessageData[6];

//	M3508s1[EMID].OffLine_Status = 0;
//	M3508s1[EMID].OffLine_Detection++;
	
}


