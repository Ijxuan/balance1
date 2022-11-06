#include "DJI_IMU.h"


DR16_Send_u DR16_T;
//由于can的数据帧最大的字节数为8
//欧拉角的一个轴的数据就是一个float类型变量，是4个字节
//那也就最多只能发两个轴
Euler_Send_u Euler_Send;
Gyro_Send_u Gyro_Send;
//接收变量
IMU_CAL_t IMU_CAL;

#if send_way == 0
/*第一种发送方式：联合体*/

//DR16
void DR16_Send_Fun(DR16_Send_u DR16_Send)
{
	//8个1字节的缓存局部变量
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
	
	//以CAN通讯发送
	CAN_SendData(&hcan2,CAN_ID_STD,DR16_Angle_SENDID,data);
}

//欧拉角
void Euler_Send_Fun(Euler_Send_u Euler_Send)
{
	//8个1字节的缓存局部变量
	uint8_t data[8];
	
	//Yaw轴angle
	data[0] = Euler_Send.Euler_Angle[0];
	data[1] = Euler_Send.Euler_Angle[1];
	data[2] = Euler_Send.Euler_Angle[2];
	data[3] = Euler_Send.Euler_Angle[3];
	
	//Pitch轴angle
	data[4] = Euler_Send.Euler_Angle[4];
	data[5] = Euler_Send.Euler_Angle[5];
	data[6] = Euler_Send.Euler_Angle[6];
	data[7] = Euler_Send.Euler_Angle[7];
	
	//以CAN通讯发送
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Angle_SENDID,data);
}
//角速度
void Gyro_Send_Fun(Gyro_Send_u Gyro_Send)
{
	//8个1字节的缓存局部变量
	uint8_t data[8];
	
	//Yaw轴angle
	data[0] = Gyro_Send.Gyro_zy[0];
	data[1] = Gyro_Send.Gyro_zy[1];
	data[2] = Gyro_Send.Gyro_zy[2];
	data[3] = Gyro_Send.Gyro_zy[3];
	
	//Pitch轴angle
	data[4] = Gyro_Send.Gyro_zy[4];
	data[5] = Gyro_Send.Gyro_zy[5];
	data[6] = Gyro_Send.Gyro_zy[6];
	data[7] = Gyro_Send.Gyro_zy[7];
	
	//以CAN通讯发送
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Gyro_SENDID,data);
}

//接收解包
void IMU_Cal_Status_Reivece(CAN_Rx_TypeDef CAN_Rx_Structure)
{
	if(CAN_Rx_Structure.CAN_RxMessage.StdId != IMU_CAL_REIID)
	{
		return;
	}
	//获取陀螺仪当前的校准状态
	IMU_CAL.real_Status = CAN_Rx_Structure.CAN_RxMessageData[0];
}
#endif

#if send_way == 1
/*第二种发送形式：指针*/
//欧拉角
void Euler_Send_Fun(float Yaw,float Pitch)
{
	//所有指针都占4个字节
	unsigned char* p[2];
	uint8_t data[8];
	
	p[0] = (unsigned char*)&Yaw;
	p[1] = (unsigned char*)&Pitch;
	
	//Yaw轴angle
	data[0] = *p[0];
	data[1] = *(p[0] + 1);
	data[2] = *(p[0] + 2);
	data[3] = *(p[0] + 3);
	
	//Pitch轴angle
	data[4] = *p[1];
	data[5] = *(p[1] + 1);
	data[6] = *(p[1] + 2);
	data[7] = *(p[1] + 3);
	
	//以CAN通讯发送
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Angle_SENDID,data);
}
//角速度
void Gyro_Send_Fun(float Gyro_z,float Gyro_y)
{
	//所有指针都占4个字节
	unsigned char* p[2];
	uint8_t data[8];
	
	p[0] = (unsigned char*)&Gyro_z;
	p[1] = (unsigned char*)&Gyro_y;
	
	//Yaw轴angle
	data[0] = *p[0];
	data[1] = *(p[0] + 1);
	data[2] = *(p[0] + 2);
	data[3] = *(p[0] + 3);
	
	//Pitch轴angle
	data[4] = *p[1];
	data[5] = *(p[1] + 1);
	data[6] = *(p[1] + 2);
	data[7] = *(p[1] + 3);
	
	//以CAN通讯发送
	CAN_SendData(&hcan2,CAN_ID_STD,DJI_C_Gyro_SENDID,data);
}

#endif
