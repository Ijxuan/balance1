#include "Vision.h"
#include "FPS_Calculate.h"
#include "usbd_cdc_if.h"
#include "DR16_RECIVE.h"
#include "my_positionPID_bate.h"
#include "MY_CLOUD_CONTROL.h"
#include "spinning_top_examine.h"
#include "Vision_Control.h"
#include "RM_JudgeSystem.h"


//#include "usb_device.h"
//#include "usbd_core.h"
//#include "usbd_desc.h"
//#include "usbd_cdc.h"
//#include "usbd_cdc_if.h"


//视觉的接收数据缓冲区
uint8_t Vision_DataBuff[Vision_BuffSize];
//视觉的接收数据结构体
VisionData_t VisionData;
//视觉的发送数据结构体
VisionSend_Cloud_t Vision_Cloud;

#if Vision_TX_NEW  //
//视觉发送数据结构体
VisionSend_Cloud_t_NEW Vision_Send_new;
#endif  //

//JC EM_R;
//#include "user_UART.h"

//USART的DMA使能 和 DMAx_Streamy数据流使能
//初始化 USART句柄 和
//补全 DMAx_Stream 变量中的 外设基地址 和 存储内存基地址
int USART_RX_DMA_ENABLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	if (huart->RxState == HAL_UART_STATE_READY)
	{

		/*输入的地址或者数据有问题的话*/
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		//初始化USART句柄的状态变量
		huart->pRxBuffPtr = pData; //将存储BUFF内存地址 赋给 USART句柄中的 pRxBuffPtr
		huart->RxXferSize = Size;  //将存储BUFF内存地址 赋给 USART句柄中的 RxXferSize

		//USART句柄的状态变量
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		

		//将外设基地址赋给源地址
		//将存储内存基地址赋给目标地址
		//使能DMA
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
		//使能USART_RX的DMA方
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	}

	else
	{
		return HAL_BUSY;
	}

	return HAL_OK;
}

//视觉的发送数据缓冲区
//帧头‘S’
//敌我双方：0红蓝 1红 2蓝
//模式：0默认 1自瞄 2大神符 3哨兵 4基地
//ID：1英雄 2工程 3步兵 6无人机 7哨兵
//帧尾‘E’
uint8_t Vision_SendBuff[5][18] = {'S', '0', '7', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0','0', '0', 'E',
								  'S', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0','0', '0', 'E',
								  'S', '0', '2', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0','0', '0', 'E',
								  'S', '0', '3', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0','0', '0', 'E',
								  'S', '0', '4', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0','0', '0', 'E'};
//									1	2	 3    4    5    6    7    8    9   10    11   12   13   14   15  16   17   18

//视觉接收函数
bool shoot_last=0;

float YAW_BC_VALUE=0;	
float YAW_BC_now=0;								  
								  
void Vision_DataReceive(uint8_t *data)
{
	#if 0
	//进行CRC校验
	uint8_t CRCBuffer = Checksum_CRC8(data, 13 - 2);

	//将视觉发送过来的13个8位数据遍历一遍
	for (uint8_t i = 0; i < 13; i++)
	{
		VisionData.RawData.VisionRawData[i] = data[i];
	}
//EM_R.step_1=1;
	//将Yaw\Pitch\Depth的高低八位合并
	VisionData.RawData.Yaw_Angle = (VisionData.RawData.VisionRawData[4] | (VisionData.RawData.VisionRawData[5] << 8));
	VisionData.RawData.Pitch_Angle = (VisionData.RawData.VisionRawData[7] | (VisionData.RawData.VisionRawData[8] << 8));
	VisionData.RawData.Depth = (VisionData.RawData.VisionRawData[9] | (VisionData.RawData.VisionRawData[10] << 8));

	//判断Yaw\Pitch的符号
	if (VisionData.RawData.Yaw_Dir == 0)
	{
		VisionData.RawData.Yaw_Angle *= -1.0f;
	}
	if (VisionData.RawData.Pitch_Dir == 0)
	{
		VisionData.RawData.Pitch_Angle *= -1.0f;
	}
//				//Yaw轴：单位 角度° 转成 机械角度(码盘值)
			Vision_RawData_Yaw_Angle = (float)VisionData.RawData.Yaw_Angle / 100.0f;
VisionData_Hand.Vision_RawData.Yaw_Angle = (float)VisionData.RawData.Yaw_Angle / 100.0f;
	
	//			//Pitch轴：
			Vision_RawData_Pitch_Angle = (float)VisionData.RawData.Pitch_Angle / 100.0f;
				VisionData_Hand.Vision_RawData.Pitch_Angle = (float)VisionData.RawData.Pitch_Angle / 100.0f;

if(abs(Vision_RawData_Pitch_Angle)>30)//PITCH轴接收到的值 绝对值 超过30,判断为错误 归零
{
	Vision_RawData_Pitch_Angle=0;
}
PITCH_TRAGET_ANGLE_TEMP=DJIC_IMU.total_pitch-Vision_RawData_Pitch_Angle;
YAW_TRAGET_ANGLE_TEMP=DJIC_IMU.total_yaw-Vision_RawData_Yaw_Angle;
	if(VisionData.RawData.Beat==1&&shoot_last==1)//连续两帧,从第二帧开始累加
	vision_shoot_times++;
	else
	vision_shoot_times=0;
	
	shoot_last=VisionData.RawData.Beat;
//EM_R.step_2=1;	
	//接收到错误的信息，则相当于无接收到消息，则无视觉作用
	//脱离视野范围，是接收到正确的消息
	//无论哪种情况都 保存最后一刻的数据，保证切换时衔接得稳定
	//CRC检验失败 或 帧头或帧尾错误
	if (CRCBuffer != VisionData.RawData.crc || VisionData.RawData.Start_Tag != 'S' || VisionData.RawData.End_Tag != 'E')
	{
		//也视为离线
//EM_R.step_3=0;
		VisionData.Offline_Flag = 1;
		return;
	}
//EM_R.step_3=1;

	//数据更新成功标志位
	VisionData.DataUpdate_Flag = 1;
	//更新成功就说明没有错误信息
	VisionData.Offline_Flag = 0;
	
	//获取视觉帧率
//	Get_FPS(&VisionData.Vision_WorldTimes, &VisionData.FPS);

	Get_FPS(&FPS_ALL.Vision.WorldTimes,   &FPS_ALL.Vision.FPS);
	
	//视觉离线检测位
	VisionData.Offline_Detec++;
	#endif
		#if 1
			//进行CRC校验
	uint8_t CRCBuffer = Checksum_CRC8(data+1, 15 - 5);
text_times++;
	//将视觉发送过来的13个8位数据遍历一遍
	for (uint8_t i = 0; i < 13; i++)
	{
		VisionData.RawData.VisionRawData[i] = data[i];
	}
		//				//Yaw轴：单位 角度° 转成 机械角度(码盘值)
			Vision_RawData_Yaw_Angle =-1.0f* (float)VisionData.RawData.Yaw_Angle ;
//			//Pitch轴：
			Vision_RawData_Pitch_Angle = (float)VisionData.RawData.Pitch_Angle ;
	#if SHOOT_HIGH_HEAT_TEXT
VisionData.RawData.Armour=1;
//VisionData.RawData.Beat=1;
	Vision_RawData_Pitch_Angle=0;
	Vision_RawData_Yaw_Angle=0;
#endif
	
	
	#if USE_MOTOR_angle==1

//	if(DR16.rc.s_left==1)
PITCH_TRAGET_ANGLE_TEMP=GM6020s[3].totalAngle-Vision_RawData_Pitch_Angle/360.0*8191;
PITCH_TRAGET_ANGLE_TEMP_EM=GM6020s[3].totalAngle-Vision_RawData_Pitch_Angle/360.0*8191.0;

#endif
	#if USE_MOTOR_angle==0
PITCH_TRAGET_ANGLE_TEMP=DJIC_IMU.total_pitch-Vision_RawData_Pitch_Angle;


#endif

if(Vision_RawData_Yaw_Angle>1.0)
YAW_BC_now=YAW_BC_VALUE;
if(Vision_RawData_Yaw_Angle<0.3&&Vision_RawData_Yaw_Angle>-0.3)
{
YAW_BC_now=0;
}
if(Vision_RawData_Yaw_Angle<-1.0)
YAW_BC_now=-YAW_BC_VALUE;
	

YAW_TRAGET_ANGLE_TEMP=DJIC_IMU.total_yaw-Vision_RawData_Yaw_Angle+YAW_BC_now;
	if(abs(Vision_RawData_Pitch_Angle)>30)//PITCH轴接收到的值 绝对值 超过30,判断为错误 归零
{
	Vision_RawData_Pitch_Angle=0;
}
    vision_beats_give_to_jia=VisionData.RawData.Beat;
	if(VisionData.RawData.Beat==1&&shoot_last==1)//连续两帧,从第二帧开始累加
	vision_shoot_times++;
	else
	vision_shoot_times=0;
	
		shoot_last=VisionData.RawData.Beat;

	if (CRCBuffer == VisionData.RawData.crc)
	crc_right=1;
	else 
	crc_right=0;
	
		Get_FPS(&FPS_ALL.Vision.WorldTimes,   &FPS_ALL.Vision.FPS);

		if (VisionData.RawData.Start_Tag != 'S' || VisionData.RawData.End_Tag != 'E')
	{
		vision_rc_error++;
		//也视为离线
//EM_R.step_3=0;

	}
	else
	{
		vision_rc_right++;
	}
		//视觉离线检测位
	VisionData.Offline_Detec++;
		#endif

}


//设置机器人的ID 和 类型
static void Vision_Set_ID_Type(uint8_t ID, uint8_t Type)
{
	for (uint8_t n = 0; n < 5; n++)
	{
		Vision_SendBuff[n][1] = ID;
		Vision_SendBuff[n][3] = Type;
	}
}
void Vision_ID_Type_Init(void)
{
//	switch (ext_game_robot_state.data.robot_id)
//	{
//	case 0:
//		Robots_Control.TeamColor = TeamColor_Red;
//		Robots_Control.Types = Types_Sentry;
		Vision_Set_ID_Type('2', '7');

}

//向视觉发送数据
static void Vision_DataSend(uint8_t *data)
{
	if (data == NULL)
		return;
	
//	CDC_Transmit_FS(data,18);
//	    CDC_Transmit_FS(Buf, *Len);

//	for (uint8_t i = 0; i < 18; i++)
//	{
//		while ((USART6->SR & 0X40) == 0);
//		USART6->DR = data[i];
//	}
	
//		for (uint8_t i = 0; i < 13; i++)
//	{
//		while ((USART6->SR & 0X40) == 0);
//		USART6->DR = data[i];
//	}//三PIN接口
		HAL_UART_Transmit_DMA(&huart6,&data[0],13);
	
	
//		for (uint8_t i = 0; i < 13; i++)
//	{
//		while ((USART6->SR & 0X40) == 0);
//		USART6->DR = data[i];
//	}
	
	
//		HAL_UART_Transmit_DMA(&huart1,&data[0],18);

//	CDC_Transmit_FS(data,18);

}
int mode_v=6;
//更新发送给视觉的数据,并发送
void Update_Vision_SendData(void)
{
	uint8_t i=3;
#if 0
	if(0)
{	
	Vision_ID_Type_Init();

	for (uint8_t i = 0; i < 5; i++)
	{


		//云台Yaw轴的角度偏差 float -> uint8_t
		Vision_SendBuff[i][4] = Vision_Cloud.VisionSend_t.Angle_Error_Data[0];
		Vision_SendBuff[i][5] = Vision_Cloud.VisionSend_t.Angle_Error_Data[1];
		Vision_SendBuff[i][6] = Vision_Cloud.VisionSend_t.Angle_Error_Data[2];
		Vision_SendBuff[i][7] = Vision_Cloud.VisionSend_t.Angle_Error_Data[3];
		//云台Pitch轴的角度偏差
		Vision_SendBuff[i][8] = Vision_Cloud.VisionSend_t.Angle_Error_Data[4];
		Vision_SendBuff[i][9] = Vision_Cloud.VisionSend_t.Angle_Error_Data[5];
		Vision_SendBuff[i][10] = Vision_Cloud.VisionSend_t.Angle_Error_Data[6];
		Vision_SendBuff[i][11] = Vision_Cloud.VisionSend_t.Angle_Error_Data[7];
		//Z的加速度
		Vision_SendBuff[i][12] = Vision_Cloud.Gyro_z_Hight;
		Vision_SendBuff[i][13] = Vision_Cloud.Gyro_z_low;
		//Y的加速度		
		Vision_SendBuff[i][14] = Vision_Cloud.Gyro_z_Hight;
		Vision_SendBuff[i][15] = Vision_Cloud.Gyro_z_low;
		//首支枪管的速度限制
		Vision_SendBuff[i][16] = 30;

	}
}	
	if(0)
{
	for (uint8_t i = 0; i < 5; i++)
	{
						Vision_SendBuff[i][0] = 'S';


		//云台Yaw轴的角度偏差 float -> uint8_t
		Vision_SendBuff[i][1] = Vision_Cloud.VisionSend_t.Angle_Error_Data[0];
		Vision_SendBuff[i][2] = Vision_Cloud.VisionSend_t.Angle_Error_Data[1];
		Vision_SendBuff[i][3] = Vision_Cloud.VisionSend_t.Angle_Error_Data[2];
		Vision_SendBuff[i][4] = Vision_Cloud.VisionSend_t.Angle_Error_Data[3];
		//云台Pitch轴的角度偏差
		Vision_SendBuff[i][5] = Vision_Cloud.VisionSend_t.Angle_Error_Data[4];
		Vision_SendBuff[i][6] = Vision_Cloud.VisionSend_t.Angle_Error_Data[5];
		Vision_SendBuff[i][7] = Vision_Cloud.VisionSend_t.Angle_Error_Data[6];
		Vision_SendBuff[i][8] = Vision_Cloud.VisionSend_t.Angle_Error_Data[7];

		//首支枪管的速度限制
		if(STATUS_complete_update_TIMES>1)
		Vision_SendBuff[i][9] = ext_game_robot_state.data.shooter_id1_17mm_speed_limit;
		else
		Vision_SendBuff[i][9] = 30;
		
				if(STATUS_complete_update_TIMES>1)
		Vision_SendBuff[i][10] = ext_game_robot_state.data.robot_id;
		else
		Vision_SendBuff[i][10] = 107;//107：蓝方哨兵机器人；7：红方哨兵机器人
			
		Vision_SendBuff[i][11] = 5;//?
				//模式：0默认 1自瞄 2大神符 3哨兵 4基地
        //'5'哨兵专用  视频录制
		
		Vision_SendBuff[i][12] = 'E';

	}
}	
#endif
	
	if(1)
{
//	for (uint8_t i = 0; i < 5; i++)
//	{
						Vision_SendBuff[i][0] = 'S';
		if(STATUS_complete_update_TIMES>1)
		Vision_SendBuff[i][1] = ext_game_robot_state.data.robot_id;
		else
		Vision_SendBuff[i][1] = 107;//107：蓝方哨兵机器人；7：红方哨兵机器人
			
		Vision_SendBuff[i][2] = mode_v;//?
		if(YAW_MOTION_STATE==12)
		{//检测到小陀螺
		Vision_SendBuff[i][2] = 5;//?
		}
		//2  5  陀螺 6预测  1基础自瞄  
//		if(stay_in_track_end_times>50&&stay_in_track_end_times<150)//在轨道末端,并且不超过1.5秒,超过1.5s可能是在轨道末端失能了
//		{
//		Vision_SendBuff[i][2] = 1;		//关掉预测
//			send_to_vision_1=1;
//		}
//		else
//		{
//			send_to_vision_1=0;
//		}
		//模式：0默认 1自瞄 2大神符 3哨兵 4基地
        //'5'哨兵专用  视频录制
		//云台Yaw轴的角度偏差 float -> uint8_t
		Vision_SendBuff[i][3] = Vision_Cloud.VisionSend_t.Angle_Error_Data[0];
		Vision_SendBuff[i][4] = Vision_Cloud.VisionSend_t.Angle_Error_Data[1];
		Vision_SendBuff[i][5] = Vision_Cloud.VisionSend_t.Angle_Error_Data[2];
		Vision_SendBuff[i][6] = Vision_Cloud.VisionSend_t.Angle_Error_Data[3];
		//云台Pitch轴的角度偏差
		Vision_SendBuff[i][7] = Vision_Cloud.VisionSend_t.Angle_Error_Data[4];
		Vision_SendBuff[i][8] = Vision_Cloud.VisionSend_t.Angle_Error_Data[5];
		Vision_SendBuff[i][9] = Vision_Cloud.VisionSend_t.Angle_Error_Data[6];
		Vision_SendBuff[i][10] = Vision_Cloud.VisionSend_t.Angle_Error_Data[7];

		//首支枪管的速度限制
		if(STATUS_complete_update_TIMES>1)
		Vision_SendBuff[i][11] = ext_game_robot_state.data.shooter_id1_17mm_speed_limit;
		else
		Vision_SendBuff[i][11] = 30;
		
		Vision_SendBuff[i][12] = 'E';

//	}
}	

		//根据攻击的模式，发给视觉
//		switch (Robots_Control.AttackTarget)
//		{
//		case ShootTarget_default:
//			Vision_DataSend(Vision_SendBuff[0]);
//			break;
//		case ShootTarget_Self_aiming:
//			Vision_DataSend(Vision_SendBuff[1]);
//			break;
//		case ShootTarget_BIG_WHEEL:
//			Vision_DataSend(Vision_SendBuff[2]);
//			break;
//		case ShootTarget_Sentry:
			Vision_DataSend(Vision_SendBuff[3]);
//			break;
//		case ShootTarget_base:
//			Vision_DataSend(Vision_SendBuff[4]);
//			break;
//		}
		
}


//DMA数据流接收的数据处理
void Vision_Handler(UART_HandleTypeDef *huart)
{

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		if (__HAL_DMA_GET_COUNTER(huart->hdmarx) == 2)
		{
			Vision_DataReceive(Vision_DataBuff);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, Vision_BuffSize);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}
