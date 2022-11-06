#include "DJI_IMU.h"


DR16_Send_u DR16_T;
//����can������֡�����ֽ���Ϊ8
//ŷ���ǵ�һ��������ݾ���һ��float���ͱ�������4���ֽ�
//��Ҳ�����ֻ�ܷ�������
Euler_Send_u Euler_Send;
Gyro_Send_u Gyro_Send;
//���ձ���
IMU_CAL_t IMU_CAL;

#if send_way == 0
/*��һ�ַ��ͷ�ʽ��������*/

//DR16
void DR16_Send_Fun(DR16_Send_u DR16_Send)
{
	//8��1�ֽڵĻ���ֲ�����
	uint8_t data[8];
	
	//CH0
	data[0] = DR16_Send.DR16_SEND[0];
	data[1] = DR16_Send.DR16_SEND[1];
	//CH1
	data[2] = DR16_Send.DR16_SEND[2];
	data[3] = DR16_Send.DR16_SEND[3];
	
	
	//CH2
	data[4] = DR16_Send.DR16_SEND[4];
	data[5] = DR16_Send.DR16_SEND[5];
	//CH3
	data[6] = DR16_Send.DR16_SEND[6];
	data[7] = DR16_Send.DR16_SEND[7];
	
	//��CANͨѶ����
	CAN_SendData(&hcan2,CAN_ID_STD,DR16_Angle_SENDID,data);
}

//ŷ����
void Euler_Send_Fun(Euler_Send_u Euler_Send)
{
	//8��1�ֽڵĻ���ֲ�����
	uint8_t data[8];
	
	//Yaw��angle
	data[0] = Euler_Send.Euler_Angle[0];
	data[1] = Euler_Send.Euler_Angle[1];
	data[2] = Euler_Send.Euler_Angle[2];
	data[3] = Euler_Send.Euler_Angle[3];
	
	//Pitch��angle
	data[4] = Euler_Send.Euler_Angle[4];
	data[5] = Euler_Send.Euler_Angle[5];
	data[6] = Euler_Send.Euler_Angle[6];
	data[7] = Euler_Send.Euler_Angle[7];
	
	//��CANͨѶ����
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Angle_SENDID,data);
}
//���ٶ�
void Gyro_Send_Fun(Gyro_Send_u Gyro_Send)
{
	//8��1�ֽڵĻ���ֲ�����
	uint8_t data[8];
	
	//Yaw��angle
	data[0] = Gyro_Send.Gyro_zy[0];
	data[1] = Gyro_Send.Gyro_zy[1];
	data[2] = Gyro_Send.Gyro_zy[2];
	data[3] = Gyro_Send.Gyro_zy[3];
	
	//Pitch��angle
	data[4] = Gyro_Send.Gyro_zy[4];
	data[5] = Gyro_Send.Gyro_zy[5];
	data[6] = Gyro_Send.Gyro_zy[6];
	data[7] = Gyro_Send.Gyro_zy[7];
	
	//��CANͨѶ����
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Gyro_SENDID,data);
}

//���ս��
void IMU_Cal_Status_Reivece(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	if(CAN_Rx_Structure.CAN_RxMessage.StdId != IMU_CAL_REIID)
	{
		return;
	}
	//��ȡ�����ǵ�ǰ��У׼״̬
	IMU_CAL.real_Status = CAN_Rx_Structure.CAN_RxMessageData[0];
}
#endif

#if send_way == 1
/*�ڶ��ַ�����ʽ��ָ��*/
//ŷ����
void Euler_Send_Fun(float Yaw,float Pitch)
{
	//����ָ�붼ռ4���ֽ�
	unsigned char* p[2];
	uint8_t data[8];
	
	p[0] = (unsigned char*)&Yaw;
	p[1] = (unsigned char*)&Pitch;
	
	//Yaw��angle
	data[0] = *p[0];
	data[1] = *(p[0] + 1);
	data[2] = *(p[0] + 2);
	data[3] = *(p[0] + 3);
	
	//Pitch��angle
	data[4] = *p[1];
	data[5] = *(p[1] + 1);
	data[6] = *(p[1] + 2);
	data[7] = *(p[1] + 3);
	
	//��CANͨѶ����
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Angle_SENDID,data);
}
//���ٶ�
void Gyro_Send_Fun(float Gyro_z,float Gyro_y)
{
	//����ָ�붼ռ4���ֽ�
	unsigned char* p[2];
	uint8_t data[8];
	
	p[0] = (unsigned char*)&Gyro_z;
	p[1] = (unsigned char*)&Gyro_y;
	
	//Yaw��angle
	data[0] = *p[0];
	data[1] = *(p[0] + 1);
	data[2] = *(p[0] + 2);
	data[3] = *(p[0] + 3);
	
	//Pitch��angle
	data[4] = *p[1];
	data[5] = *(p[1] + 1);
	data[6] = *(p[1] + 2);
	data[7] = *(p[1] + 3);
	
	//��CANͨѶ����
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Gyro_SENDID,data);
}

#endif
