#include "MF9025.h"

MF9025_t MF9025[2];
 int mf9025_tgg_speed;//ʵ��������ת��3300
 int send_to_MF9025_TEST;//�ر�ת�����440  6.4A  �̶������£�ת�ٲ���40-60
void MF9025_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure)
{
    uint32_t EMID;
    EMID = (uint32_t)(CAN_Rx_Structure.CAN_RxMessage.StdId - MF9025_READID_START);

    MF9025[EMID].command = CAN_Rx_Structure.CAN_RxMessageData[0]; // �����ֽ�
    MF9025[EMID].temperature = CAN_Rx_Structure.CAN_RxMessageData[1]; // ����¶�
    MF9025[EMID].real_torque_Current = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[3] << 8 | CAN_Rx_Structure.CAN_RxMessageData[2]); // ת�ص���
    MF9025[EMID].realSpeed = (int16_t)(CAN_Rx_Structure.CAN_RxMessageData[5] << 8 | CAN_Rx_Structure.CAN_RxMessageData[4]); // ����ٶ�
    MF9025[EMID].real_encoder_Angle = (uint16_t)(CAN_Rx_Structure.CAN_RxMessageData[7] << 8 | CAN_Rx_Structure.CAN_RxMessageData[6]); // ������λ��



}
//ͨ���㲥ģʽ��������
void MF9025_setTorque(int16_t Iid1,int16_t Iid2,int16_t Iid3,int16_t Iid4)
{
	uint8_t data[8];
	
	
	data[0] = Iid1;				
	data[1] = Iid1 >> 8;							
	data[2] = Iid2;
	data[3] = Iid2 >> 8;
	data[4] = Iid3;
	data[5] = Iid3 >> 8;
	data[6] = Iid4;
	data[7] = Iid4 >> 8;
	
	CAN_SendData(&hcan1,CAN_ID_STD,MF9025_Broadcast_Mode_id,data);
	
}






