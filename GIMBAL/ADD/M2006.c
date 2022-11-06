

#include "M2006.h"
#include "M3508.h"

void M2006_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	
		if(CAN_Rx_Structure.CAN_RxMessage.StdId != M2006_SENDID+2)
		return;
	
	uint32_t EMID;				
	EMID = (uint32_t)(CAN_Rx_Structure.CAN_RxMessage.StdId - M2006_READID_START);

//	CAN_Rx_Structure.CAN_RxMessage = ptr->angle;
	M3508s[EMID].realAngle = (uint16_t)(CAN_Rx_Structure.CAN_RxMessageData[0]<<8 | CAN_Rx_Structure.CAN_RxMessageData[1]) ;
	M3508s[EMID].realSpeed  = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[2]<<8 | CAN_Rx_Structure.CAN_RxMessageData[3]);
	M3508s[EMID].realCurrent = (CAN_Rx_Structure.CAN_RxMessageData[4]<<8 | CAN_Rx_Structure.CAN_RxMessageData[5])*5.f/16384.f;

	
	
	if(M3508s[EMID].realAngle - M3508s[EMID].last_angle > 4096)
		M3508s[EMID].turnCount --;
	else if (M3508s[EMID].realAngle - M3508s[EMID].last_angle < -4096)
		M3508s[EMID].turnCount ++;
	
	M3508s[EMID].totalAngle = M3508s[EMID].turnCount * 8192+ M3508s[EMID].realAngle;
	
		 M3508s[EMID].last_angle=M3508s[EMID].realAngle;

	
	
	
}








































