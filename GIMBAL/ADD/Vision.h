#ifndef VISION_H
#define VISION_H

#include "main.h"
//#include "PID_Position.h"
//#include "PID_Increment.h"
#include <stdio.h>
//#include "CRC.h"
#include "usart.h"
//#include "Robots_Detection.h"
//#include "User_typedef.h"
#include "CRC.h" 

#pragma anon_unions


#define Vision_BuffSize (13 + 2) //�Ӿ��������ݻ���������
extern uint8_t Vision_DataBuff[Vision_BuffSize];

#define VisionPage_Width 1280
#define VisionPage_Height 800

#define Vision_TX_NEW 1

#if Vision_TX_NEW  //
//�Ӿ��������ݽṹ��
typedef struct
{
	union
	{
		struct
		{
			float YawAngle_Error;	//������YAW�ǶȲ�
			float PitchAngle_Error; //������Pitch�ǶȲ�
		};
		uint8_t Angle_Error_Data[8];
	} VisionSend_t;



} VisionSend_Cloud_t_NEW;
typedef struct
{
	uint8_t start_tag;
	uint8_t robot_id;
	/*
	������ ID��
1��Ӣ��(��)��
2������(��)��
3/4/5������(��)��
6������(��)��
7���ڱ�(��)��
9���״�վ���죩��

101��Ӣ��(��)��
102������(��)��
103/104/105������(��)��
106������(��)��
107���ڱ�(��)��
109���״�վ��������*/
	uint8_t mode;
	float yaw_angle;
	float pit_angle;
	uint8_t shoot_speed;
	uint8_t end_tag;

}VisionSend_Pack_t;

#endif  //





typedef struct
{
	float X;
	float Y;

} XY_t;

#pragma pack(1)

typedef struct
{
	struct
	{
		#if 0
		union
		{
			struct
			{
				char Start_Tag; //֡ͷ

				uint8_t Armour; //�Ƿ�ʶ��װ�װ�
				uint8_t Beat; //�Ƿ񹥻����ų�����2�ţ�

				uint8_t Yaw_Dir;		//Yaw�᷽��
				uint8_t Yaw_Angle_Low;	//Yaw��ǶȵͰ�λ
				uint8_t Yaw_Angle_High; //Yaw��Ƕȸ߰�λ

				uint8_t Pitch_Dir;		  //Pitch�᷽��
				uint8_t Pitch_Angle_Low;  //Pitch��ǶȵĵͰ�λ
				uint8_t Pitch_Angle_High; //Pitch��Ƕȵĸ߰�λ

				uint8_t Depth_Low;	//��ȵĵͰ�λ
				uint8_t Depth_High; //��ȵĸ߰�λ

				uint8_t crc; //CRCУ��λ

				char End_Tag; //֡β
			};
			uint8_t VisionRawData[13];
		};

		float Yaw_Angle;				//Yaw��ĽǶ�
		float Pitch_Angle;				//Pitch��ĽǶ�
		float Depth;					//���
	#endif
	#if 1
			union
		{
			struct
			{
				char Start_Tag; //֡ͷ   0

				uint8_t Armour; //�Ƿ�ʶ��װ�װ�   1
				uint8_t Beat; //�Ƿ񹥻����ų�����2�ţ�  2

		float Yaw_Angle;				//Yaw��ĽǶ�   3 4 5 6


		float Pitch_Angle;				//Pitch��ĽǶ� 7 8 9 10
				

//        uint16_t  Depth;                             //   12 13

				uint8_t crc; //CRCУ��λ  uint16_t               11

				char End_Tag; //֡β                 14
			};
			uint8_t VisionRawData[13];
		};
	
	#endif
	} RawData; //�Ӿ���Э�� ����һ֡�����ݽṹ��

	uint32_t Offline_Detec;
	uint8_t Offline_Flag;					//���ݴ����־λ
	uint8_t DataUpdate_Flag;				//���ݸ��±�־λ
	
	//�Ӿ��Ľ�������֡��
//	WorldTime_RxTypedef Vision_WorldTimes;
	uint32_t FPS;							//֡��

} VisionData_t;
#pragma pack()
//#pragma pack(1)

//�Ӿ��������ݽṹ��
typedef struct
{
	union
	{
		struct
		{
			float YawAngle_Error;	//������YAW�ǶȲ�
			float PitchAngle_Error; //������Pitch�ǶȲ�
		};
		uint8_t Angle_Error_Data[8];
	} VisionSend_t;

	int Gyro_z;			  //�����Ǽ��ٶ�С�������λ
	uint8_t Gyro_z_Hight; //�����Ǽ��ٶ�С�������λ�߰�λ
	uint8_t Gyro_z_low;	  //�����Ǽ��ٶ�С�������λ�Ͱ�λ
	uint8_t Gyro_y_Hight; //�����Ǽ��ٶ�С�������λ�߰�λ
	uint8_t Gyro_y_low;	  //�����Ǽ��ٶ�С�������λ�Ͱ�λ

} VisionSend_Cloud_t;
//#pragma pack()


extern VisionData_t VisionData;
extern VisionSend_Cloud_t Vision_Cloud;
extern int mode_v;

void Vision_DataReceive(uint8_t *data);
void Vision_Handler(UART_HandleTypeDef *huart);

void Vision_ID_Type_Init(void);
void Update_Vision_SendData(void);
int USART_RX_DMA_ENABLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);



#endif
