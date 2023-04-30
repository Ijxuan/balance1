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


//�Ӿ��Ľ������ݻ�����
uint8_t Vision_DataBuff[Vision_BuffSize];
//�Ӿ��Ľ������ݽṹ��
VisionData_t VisionData;
//�Ӿ��ķ������ݽṹ��
VisionSend_Cloud_t Vision_Cloud;

#if Vision_TX_NEW  //
//�Ӿ��������ݽṹ��
VisionSend_Cloud_t_NEW Vision_Send_new;
#endif  //

//JC EM_R;
//#include "user_UART.h"

//USART��DMAʹ�� �� DMAx_Streamy������ʹ��
//��ʼ�� USART��� ��
//��ȫ DMAx_Stream �����е� �������ַ �� �洢�ڴ����ַ
int USART_RX_DMA_ENABLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	if (huart->RxState == HAL_UART_STATE_READY)
	{

		/*����ĵ�ַ��������������Ļ�*/
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		//��ʼ��USART�����״̬����
		huart->pRxBuffPtr = pData; //���洢BUFF�ڴ��ַ ���� USART����е� pRxBuffPtr
		huart->RxXferSize = Size;  //���洢BUFF�ڴ��ַ ���� USART����е� RxXferSize

		//USART�����״̬����
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		

		//���������ַ����Դ��ַ
		//���洢�ڴ����ַ����Ŀ���ַ
		//ʹ��DMA
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
		//ʹ��USART_RX��DMA��
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	}

	else
	{
		return HAL_BUSY;
	}

	return HAL_OK;
}

//�Ӿ��ķ������ݻ�����
//֡ͷ��S��
//����˫����0���� 1�� 2��
//ģʽ��0Ĭ�� 1���� 2����� 3�ڱ� 4����
//ID��1Ӣ�� 2���� 3���� 6���˻� 7�ڱ�
//֡β��E��
uint8_t Vision_SendBuff[5][18] = {'S', '0', '7', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0','0', '0', 'E',
								  'S', '0', '1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0','0', '0', 'E',
								  'S', '0', '2', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0','0', '0', 'E',
								  'S', '0', '3', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0','0', '0', 'E',
								  'S', '0', '4', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0','0', '0', 'E'};
//									1	2	 3    4    5    6    7    8    9   10    11   12   13   14   15  16   17   18

//�Ӿ����պ���
bool shoot_last=0;

float YAW_BC_VALUE=0;	
float YAW_BC_now=0;								  
								  
void Vision_DataReceive(uint8_t *data)
{
	#if 0
	//����CRCУ��
	uint8_t CRCBuffer = Checksum_CRC8(data, 13 - 2);

	//���Ӿ����͹�����13��8λ���ݱ���һ��
	for (uint8_t i = 0; i < 13; i++)
	{
		VisionData.RawData.VisionRawData[i] = data[i];
	}
//EM_R.step_1=1;
	//��Yaw\Pitch\Depth�ĸߵͰ�λ�ϲ�
	VisionData.RawData.Yaw_Angle = (VisionData.RawData.VisionRawData[4] | (VisionData.RawData.VisionRawData[5] << 8));
	VisionData.RawData.Pitch_Angle = (VisionData.RawData.VisionRawData[7] | (VisionData.RawData.VisionRawData[8] << 8));
	VisionData.RawData.Depth = (VisionData.RawData.VisionRawData[9] | (VisionData.RawData.VisionRawData[10] << 8));

	//�ж�Yaw\Pitch�ķ���
	if (VisionData.RawData.Yaw_Dir == 0)
	{
		VisionData.RawData.Yaw_Angle *= -1.0f;
	}
	if (VisionData.RawData.Pitch_Dir == 0)
	{
		VisionData.RawData.Pitch_Angle *= -1.0f;
	}
//				//Yaw�᣺��λ �Ƕȡ� ת�� ��е�Ƕ�(����ֵ)
			Vision_RawData_Yaw_Angle = (float)VisionData.RawData.Yaw_Angle / 100.0f;
VisionData_Hand.Vision_RawData.Yaw_Angle = (float)VisionData.RawData.Yaw_Angle / 100.0f;
	
	//			//Pitch�᣺
			Vision_RawData_Pitch_Angle = (float)VisionData.RawData.Pitch_Angle / 100.0f;
				VisionData_Hand.Vision_RawData.Pitch_Angle = (float)VisionData.RawData.Pitch_Angle / 100.0f;

if(abs(Vision_RawData_Pitch_Angle)>30)//PITCH����յ���ֵ ����ֵ ����30,�ж�Ϊ���� ����
{
	Vision_RawData_Pitch_Angle=0;
}
PITCH_TRAGET_ANGLE_TEMP=DJIC_IMU.total_pitch-Vision_RawData_Pitch_Angle;
YAW_TRAGET_ANGLE_TEMP=DJIC_IMU.total_yaw-Vision_RawData_Yaw_Angle;
	if(VisionData.RawData.Beat==1&&shoot_last==1)//������֡,�ӵڶ�֡��ʼ�ۼ�
	vision_shoot_times++;
	else
	vision_shoot_times=0;
	
	shoot_last=VisionData.RawData.Beat;
//EM_R.step_2=1;	
	//���յ��������Ϣ�����൱���޽��յ���Ϣ�������Ӿ�����
	//������Ұ��Χ���ǽ��յ���ȷ����Ϣ
	//������������� �������һ�̵����ݣ���֤�л�ʱ�νӵ��ȶ�
	//CRC����ʧ�� �� ֡ͷ��֡β����
	if (CRCBuffer != VisionData.RawData.crc || VisionData.RawData.Start_Tag != 'S' || VisionData.RawData.End_Tag != 'E')
	{
		//Ҳ��Ϊ����
//EM_R.step_3=0;
		VisionData.Offline_Flag = 1;
		return;
	}
//EM_R.step_3=1;

	//���ݸ��³ɹ���־λ
	VisionData.DataUpdate_Flag = 1;
	//���³ɹ���˵��û�д�����Ϣ
	VisionData.Offline_Flag = 0;
	
	//��ȡ�Ӿ�֡��
//	Get_FPS(&VisionData.Vision_WorldTimes, &VisionData.FPS);

	Get_FPS(&FPS_ALL.Vision.WorldTimes,   &FPS_ALL.Vision.FPS);
	
	//�Ӿ����߼��λ
	VisionData.Offline_Detec++;
	#endif
		#if 1
			//����CRCУ��
	uint8_t CRCBuffer = Checksum_CRC8(data+1, 15 - 5);
text_times++;
	//���Ӿ����͹�����13��8λ���ݱ���һ��
	for (uint8_t i = 0; i < 13; i++)
	{
		VisionData.RawData.VisionRawData[i] = data[i];
	}
		//				//Yaw�᣺��λ �Ƕȡ� ת�� ��е�Ƕ�(����ֵ)
			Vision_RawData_Yaw_Angle =-1.0f* (float)VisionData.RawData.Yaw_Angle ;
//			//Pitch�᣺
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
	if(abs(Vision_RawData_Pitch_Angle)>30)//PITCH����յ���ֵ ����ֵ ����30,�ж�Ϊ���� ����
{
	Vision_RawData_Pitch_Angle=0;
}
    vision_beats_give_to_jia=VisionData.RawData.Beat;
	if(VisionData.RawData.Beat==1&&shoot_last==1)//������֡,�ӵڶ�֡��ʼ�ۼ�
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
		//Ҳ��Ϊ����
//EM_R.step_3=0;

	}
	else
	{
		vision_rc_right++;
	}
		//�Ӿ����߼��λ
	VisionData.Offline_Detec++;
		#endif

}


//���û����˵�ID �� ����
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

//���Ӿ���������
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
//	}//��PIN�ӿ�
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
//���·��͸��Ӿ�������,������
void Update_Vision_SendData(void)
{
	uint8_t i=3;
#if 0
	if(0)
{	
	Vision_ID_Type_Init();

	for (uint8_t i = 0; i < 5; i++)
	{


		//��̨Yaw��ĽǶ�ƫ�� float -> uint8_t
		Vision_SendBuff[i][4] = Vision_Cloud.VisionSend_t.Angle_Error_Data[0];
		Vision_SendBuff[i][5] = Vision_Cloud.VisionSend_t.Angle_Error_Data[1];
		Vision_SendBuff[i][6] = Vision_Cloud.VisionSend_t.Angle_Error_Data[2];
		Vision_SendBuff[i][7] = Vision_Cloud.VisionSend_t.Angle_Error_Data[3];
		//��̨Pitch��ĽǶ�ƫ��
		Vision_SendBuff[i][8] = Vision_Cloud.VisionSend_t.Angle_Error_Data[4];
		Vision_SendBuff[i][9] = Vision_Cloud.VisionSend_t.Angle_Error_Data[5];
		Vision_SendBuff[i][10] = Vision_Cloud.VisionSend_t.Angle_Error_Data[6];
		Vision_SendBuff[i][11] = Vision_Cloud.VisionSend_t.Angle_Error_Data[7];
		//Z�ļ��ٶ�
		Vision_SendBuff[i][12] = Vision_Cloud.Gyro_z_Hight;
		Vision_SendBuff[i][13] = Vision_Cloud.Gyro_z_low;
		//Y�ļ��ٶ�		
		Vision_SendBuff[i][14] = Vision_Cloud.Gyro_z_Hight;
		Vision_SendBuff[i][15] = Vision_Cloud.Gyro_z_low;
		//��֧ǹ�ܵ��ٶ�����
		Vision_SendBuff[i][16] = 30;

	}
}	
	if(0)
{
	for (uint8_t i = 0; i < 5; i++)
	{
						Vision_SendBuff[i][0] = 'S';


		//��̨Yaw��ĽǶ�ƫ�� float -> uint8_t
		Vision_SendBuff[i][1] = Vision_Cloud.VisionSend_t.Angle_Error_Data[0];
		Vision_SendBuff[i][2] = Vision_Cloud.VisionSend_t.Angle_Error_Data[1];
		Vision_SendBuff[i][3] = Vision_Cloud.VisionSend_t.Angle_Error_Data[2];
		Vision_SendBuff[i][4] = Vision_Cloud.VisionSend_t.Angle_Error_Data[3];
		//��̨Pitch��ĽǶ�ƫ��
		Vision_SendBuff[i][5] = Vision_Cloud.VisionSend_t.Angle_Error_Data[4];
		Vision_SendBuff[i][6] = Vision_Cloud.VisionSend_t.Angle_Error_Data[5];
		Vision_SendBuff[i][7] = Vision_Cloud.VisionSend_t.Angle_Error_Data[6];
		Vision_SendBuff[i][8] = Vision_Cloud.VisionSend_t.Angle_Error_Data[7];

		//��֧ǹ�ܵ��ٶ�����
		if(STATUS_complete_update_TIMES>1)
		Vision_SendBuff[i][9] = ext_game_robot_state.data.shooter_id1_17mm_speed_limit;
		else
		Vision_SendBuff[i][9] = 30;
		
				if(STATUS_complete_update_TIMES>1)
		Vision_SendBuff[i][10] = ext_game_robot_state.data.robot_id;
		else
		Vision_SendBuff[i][10] = 107;//107�������ڱ������ˣ�7���췽�ڱ�������
			
		Vision_SendBuff[i][11] = 5;//?
				//ģʽ��0Ĭ�� 1���� 2����� 3�ڱ� 4����
        //'5'�ڱ�ר��  ��Ƶ¼��
		
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
		Vision_SendBuff[i][1] = 107;//107�������ڱ������ˣ�7���췽�ڱ�������
			
		Vision_SendBuff[i][2] = mode_v;//?
		if(YAW_MOTION_STATE==12)
		{//��⵽С����
		Vision_SendBuff[i][2] = 5;//?
		}
		//2  5  ���� 6Ԥ��  1��������  
//		if(stay_in_track_end_times>50&&stay_in_track_end_times<150)//�ڹ��ĩ��,���Ҳ�����1.5��,����1.5s�������ڹ��ĩ��ʧ����
//		{
//		Vision_SendBuff[i][2] = 1;		//�ص�Ԥ��
//			send_to_vision_1=1;
//		}
//		else
//		{
//			send_to_vision_1=0;
//		}
		//ģʽ��0Ĭ�� 1���� 2����� 3�ڱ� 4����
        //'5'�ڱ�ר��  ��Ƶ¼��
		//��̨Yaw��ĽǶ�ƫ�� float -> uint8_t
		Vision_SendBuff[i][3] = Vision_Cloud.VisionSend_t.Angle_Error_Data[0];
		Vision_SendBuff[i][4] = Vision_Cloud.VisionSend_t.Angle_Error_Data[1];
		Vision_SendBuff[i][5] = Vision_Cloud.VisionSend_t.Angle_Error_Data[2];
		Vision_SendBuff[i][6] = Vision_Cloud.VisionSend_t.Angle_Error_Data[3];
		//��̨Pitch��ĽǶ�ƫ��
		Vision_SendBuff[i][7] = Vision_Cloud.VisionSend_t.Angle_Error_Data[4];
		Vision_SendBuff[i][8] = Vision_Cloud.VisionSend_t.Angle_Error_Data[5];
		Vision_SendBuff[i][9] = Vision_Cloud.VisionSend_t.Angle_Error_Data[6];
		Vision_SendBuff[i][10] = Vision_Cloud.VisionSend_t.Angle_Error_Data[7];

		//��֧ǹ�ܵ��ٶ�����
		if(STATUS_complete_update_TIMES>1)
		Vision_SendBuff[i][11] = ext_game_robot_state.data.shooter_id1_17mm_speed_limit;
		else
		Vision_SendBuff[i][11] = 30;
		
		Vision_SendBuff[i][12] = 'E';

//	}
}	

		//���ݹ�����ģʽ�������Ӿ�
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


//DMA���������յ����ݴ���
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
