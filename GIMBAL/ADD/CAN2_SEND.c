#include"CAN2_SEND.h"

uint8_t SEND_MIT[8];  // 
int CAN2_SEND_TIMES=0;
int test_MIT_ID_2=0x01;
void CAN2_SEND_TO_MIT()
{
//		for(int i=0;i<8;i++ )//0��8λ��Ч,һ��9λ
//{
//	SEND_MIT[i]=0xFF;
//}
SEND_MIT[7]=0xFD;//ʧ��
//SEND_MIT[7]=0xFC;//�ǵ��ģʽ,���ؿ���

	
	    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
            buf[7] = 0xFC;

CAN_SendData(&hcan2,CAN_ID_STD,test_MIT_ID_2,buf);
CAN2_SEND_TIMES++;

}





