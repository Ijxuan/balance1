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


#define Vision_BuffSize (13 + 2) //视觉接收数据缓冲区长度
extern uint8_t Vision_DataBuff[Vision_BuffSize];

#define VisionPage_Width 1280
#define VisionPage_Height 800

#define Vision_TX_NEW 1

#if Vision_TX_NEW  //
//视觉发送数据结构体
typedef struct
{
	union
	{
		struct
		{
			float YawAngle_Error;	//陀螺仪YAW角度差
			float PitchAngle_Error; //陀螺仪Pitch角度差
		};
		uint8_t Angle_Error_Data[8];
	} VisionSend_t;



} VisionSend_Cloud_t_NEW;
typedef struct
{
	uint8_t start_tag;
	uint8_t robot_id;
	/*
	机器人 ID：
1，英雄(红)；
2，工程(红)；
3/4/5，步兵(红)；
6，空中(红)；
7，哨兵(红)；
9，雷达站（红）；

101，英雄(蓝)；
102，工程(蓝)；
103/104/105，步兵(蓝)；
106，空中(蓝)；
107，哨兵(蓝)；
109，雷达站（蓝）。*/
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
				char Start_Tag; //帧头

				uint8_t Armour; //是否识别到装甲板
				uint8_t Beat; //是否攻击（排除工程2号）

				uint8_t Yaw_Dir;		//Yaw轴方向
				uint8_t Yaw_Angle_Low;	//Yaw轴角度低八位
				uint8_t Yaw_Angle_High; //Yaw轴角度高八位

				uint8_t Pitch_Dir;		  //Pitch轴方向
				uint8_t Pitch_Angle_Low;  //Pitch轴角度的低八位
				uint8_t Pitch_Angle_High; //Pitch轴角度的高八位

				uint8_t Depth_Low;	//深度的低八位
				uint8_t Depth_High; //深度的高八位

				uint8_t crc; //CRC校验位

				char End_Tag; //帧尾
			};
			uint8_t VisionRawData[13];
		};

		float Yaw_Angle;				//Yaw轴的角度
		float Pitch_Angle;				//Pitch轴的角度
		float Depth;					//深度
	#endif
	#if 1
			union
		{
			struct
			{
				char Start_Tag; //帧头   0

				uint8_t Armour; //是否识别到装甲板   1
				uint8_t Beat; //是否攻击（排除工程2号）  2

		float Yaw_Angle;				//Yaw轴的角度   3 4 5 6


		float Pitch_Angle;				//Pitch轴的角度 7 8 9 10
				

//        uint16_t  Depth;                             //   12 13

				uint8_t crc; //CRC校验位  uint16_t               11

				char End_Tag; //帧尾                 14
			};
			uint8_t VisionRawData[13];
		};
	
	#endif
	} RawData; //视觉的协议 接收一帧的数据结构体

	uint32_t Offline_Detec;
	uint8_t Offline_Flag;					//数据错误标志位
	uint8_t DataUpdate_Flag;				//数据更新标志位
	
	//视觉的接收数据帧率
//	WorldTime_RxTypedef Vision_WorldTimes;
	uint32_t FPS;							//帧率

} VisionData_t;
#pragma pack()
//#pragma pack(1)

//视觉发送数据结构体
typedef struct
{
	union
	{
		struct
		{
			float YawAngle_Error;	//陀螺仪YAW角度差
			float PitchAngle_Error; //陀螺仪Pitch角度差
		};
		uint8_t Angle_Error_Data[8];
	} VisionSend_t;

	int Gyro_z;			  //陀螺仪加速度小数点后两位
	uint8_t Gyro_z_Hight; //陀螺仪加速度小数点后两位高八位
	uint8_t Gyro_z_low;	  //陀螺仪加速度小数点后两位低八位
	uint8_t Gyro_y_Hight; //陀螺仪加速度小数点后两位高八位
	uint8_t Gyro_y_low;	  //陀螺仪加速度小数点后两位低八位

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
