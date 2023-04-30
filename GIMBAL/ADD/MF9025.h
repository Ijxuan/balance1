#ifndef Mf9025_H
#define Mf9025_H

#include "main.h"
//#include "BSP_CAN.h"
#include "user_can.h"

#define MF9025_READID_START 0x141 //CAN���յ� MF9025�ĵ�� ��ʼID
#define MF9025_Broadcast_Mode_id 0x280 //�㲥ģʽ���Ʊ�ʶ��



typedef struct
{
	uint8_t command;	 //�������ĵ���¶�
	uint16_t real_encoder_Angle;	 //�������Ļ�е�Ƕ�
	uint16_t last_angle; //�������Ļ�е�Ƕ�
	int16_t realSpeed;	 //���������ٶ�
	int16_t real_torque_Current; //��������ʵ��ת�ص���
	uint8_t temperature;	 //�������ĵ���¶�
	
	float OutputCurrent; //M3508�������
	uint16_t targetAngle; //Ŀ��Ƕ�
	float targetSpeed;  //Ŀ���ٶ�
	int16_t turnCount;	//ת����Ȧ��
	float totalAngle; //ת�����ܽǶ�
	uint16_t OffLine_Detection; //���߼��
	uint8_t OffLine_Status;		//���߱�־λ

} MF9025_t;
extern  MF9025_t MF9025[2];
extern  int send_to_MF9025_TEST;
extern int mf9025_tgg_speed;


void MF9025_getInfo(CAN_Rx_TypeDef CAN_Rx_Structure);

void MF9025_setTorque(int16_t Iid1,int16_t Iid2,int16_t Iid3,int16_t Iid4);








#endif
