#include "DR16_RECIVE.h"
#include "stdint.h"
#include "usbd_cdc_if.h"

#include "usart.h"
#include "my_positionPID_bate.h"
//#include "GM6020.h"
//#include "DJI_C_IMU.h"
#include "M3508.h"
//#include "MY_CHASSIS_CONTROL.h"
#include "my_IncrementPID_bate.h"
#include "Vision.h"
#include "MY_CLOUD_CONTROL.h"
#include "calibrate_task.h"
#include "MY_SHOOT_CONTROL.h"

#include "bmi088driver.h"
#include "spinning_top_examine.h"
#include "Vision_Control.h"

//#include "GM6020_Motor.h"
//#include "control.h"

static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
ext_shoot_data_t ext_shoot_data;
ext_robot_hurt_t ext_robot_hurt;     
ext_game_robot_status_t ext_game_robot_state;//״̬  ʣ��Ѫ��
ext_power_heat_data_t ext_power_heat_data;//���� ����
ext_game_robot_HP_t ext_game_robot_HP;//�������л����˵�Ѫ��
ext_game_status_t      ext_game_status;//�����׶�  ׼�� ����ʱ ��ʼ......
uint8_t CHASSIS_place[8];
uint8_t JSBuffer[8];

//uint8_t DR16Buffer[DR16BufferNumber];
uint8_t DR16Buffer[22];

DR16_t DR16 = DR16_GroundInit;


uint8_t testdatatosend[50];//������λ����������

void usart1_dr16_init(void)
{
	//__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
//HAL_DMA_Start(huart1,(uint32_t)&USART1->DR,(uint32_t)DR16Buffer,DR16BufferNumber);
	
//		/*��ձ�־λȻ��ʹ��USART���ж�*/
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	__HAL_UART_ENABLE(&huart3);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
//	/*����DMA���䣨���ǲ�����DMA�жϣ�*/
	USART_Receive_DMA_NO_IT(&huart3,DR16Buffer,DR16BufferNumber);
}


//		&huart1
void DR_16hander(UART_HandleTypeDef *huart)
	
{
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
	   __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		//if(DR16BufferNumber - DMA_GET_COUNTER(huart->hdmarx->Instance) == DR16BufferTruthNumber)
		if (__HAL_DMA_GET_COUNTER(huart->hdmarx) == DR16BufferLastNumber)
		{
			DR16.DR16_Process(DR16Buffer);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, DR16BufferNumber);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
	
	
	
	
	
	
}


void DR16_Process(uint8_t *pData)
{
	if (pData == NULL)
	{
		return;
	}
	DR16.rc.ch0 = (pData[0] | (pData[1] << 8)) & 0x07FF;
	DR16.rc.ch0 -= 1024;

	DR16.rc.ch1 = ((pData[1] >> 3) | (pData[2] << 5)) & 0x07FF;
		DR16.rc.ch1 -= 1024;

	DR16.rc.ch2 = ((pData[2] >> 6) | (pData[3] << 2) | (pData[4] << 10)) & 0x07FF;
		DR16.rc.ch2 -= 1024;

	DR16.rc.ch3 = ((pData[4] >> 1) | (pData[5] << 7)) & 0x07FF;
		DR16.rc.ch3 -= 1024;
	DR16.rc.s_left = ((pData[5] >> 4) & 0x000C) >> 2;
	DR16.rc.s_right = ((pData[5] >> 4) & 0x0003);
	DR16.mouse.x = (pData[6]) | (pData[7] << 8);
	DR16.mouse.y = (pData[8]) | (pData[9] << 8);
	DR16.mouse.z = (pData[10]) | (pData[11] << 8);
	DR16.mouse.keyLeft = pData[12];
	DR16.mouse.keyRight = pData[13];
	DR16.keyBoard.key_code = pData[14] | (pData[15] << 8);

	//your control code ��.
	DR16.rc.ch4_DW = (pData[16] | (pData[17] << 8)) & 0x07FF;
		DR16.rc.ch4_DW -= 1024;
	ch4_DW_total_2+=DR16.rc.ch4_DW;
	DR16.infoUpdateFrame++;


	/* prevent remote control zero deviation */
	if (DR16.rc.ch0 <= 5 && DR16.rc.ch0 >= -5)
		DR16.rc.ch0 = 0;
	if (DR16.rc.ch1 <= 5 && DR16.rc.ch1 >= -5)
		DR16.rc.ch1 = 0;
	if (DR16.rc.ch2 <= 20 && DR16.rc.ch2 >= -20)
		DR16.rc.ch2 = 0;
	if (DR16.rc.ch3 <= 20 && DR16.rc.ch3 >= -20)
		DR16.rc.ch3 = 0;
	if (DR16.rc.ch4_DW <= 20 && DR16.rc.ch4_DW >= -20)
		DR16.rc.ch4_DW = 0;
	
	
	CH0_TOTAL+=DR16.rc.ch0;
	CH1_TOTAL+=DR16.rc.ch1;
    CH2_TOTAL+=DR16.rc.ch2;
	CH3_TOTAL+=DR16.rc.ch3;
//				if(DR16.rc.s_left==3)
////				targe_angle=-20000;
//				if(DR16.rc.s_left==1)
////				targe_angle=20000;
//				if(DR16.rc.s_left==2)
//				targe_angle+=(DR16.rc.ch0/660.0)*300;

//				mubiaosudu3=(DR16.rc.ch1/660.0)*300;

//		if (DR16.rc.ch3 >600 )
//targe_angle=20000;
//		if (DR16.rc.ch3 <-600 )
//targe_angle=-20000;
					

	
//targe_angle+=(DR16.rc.ch3/660.0)*300;
//	DR16_Export_data.ControlSwitch.Left = (RemotePole_e)DR16.rc.s_left;
//	DR16_Export_data.ControlSwitch.Right = (RemotePole_e)DR16.rc.s_right;
//	
///* 	RemoteMode_Update();//�Կ�����Դ���˶�ģʽ���и��¡�*/
//	RemoteControl_Update();//����������˶�Ŀ��ֵ�� 

			}



/**
  * @Data    2019-02-19 15:46
  * @brief   USART_DMA���տ������ض���
  * @param   void
  * @retval  void
  */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{

		/*��⵱ǰhuart״̬*/
		if(huart->RxState == HAL_UART_STATE_READY)
		{
			/*����ĵ�ַ��������������Ļ�*/
			if((pData == NULL) || (Size == 0))
			{
					return HAL_ERROR;
			}
			
			/*huart�����Ӧ��Rx�����ض���*/
			huart->pRxBuffPtr = pData;
			huart->RxXferSize = Size;
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			
			/*����huart1�ϵ�RX_DMA*/
			HAL_DMA_Start(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)pData,Size);
			
			/*ֻ������ӦDMA�����Rx���ܣ�����ǿ���Tx�Ļ�����USART_CR3_DMAT��*/
			SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
			
			
		}
		else
		{
			return HAL_BUSY;
		}

		return HAL_OK;
}

int16_t data1=1;
int16_t data2=2;
int32_t data3=3;
//int16_t 
uint8_t sumcheck = 0;
uint8_t addcheck = 0;
int32_t data4=4;

uint8_t i=0;
uint8_t p=0; 
int32_t send_d_32[8];
int16_t send_d_16[3];

int32_t send_data1=1;
int32_t send_data2=2;

int32_t send_data3=3;
int32_t send_data4=4;
int32_t send_data5=5;

int16_t send_data6=6;
int16_t send_data7=7;

int32_t send_data8=8;
int32_t send_data9=9;
int32_t send_data10=10;

int16_t send_data11=11;

/*����Ŀ��Ƕȣ���ǰ�Ƕ�-int32_t
λ�û�P_OUT,I_OUT,D_OU-Tint32_t
�ٶȻ�P_OUT,I_OUT,D_OUT-int32_t

Ŀ���ٶȣ���ǰ�ٶ�-int16_t
����������ֵ-int16_t
           28  6
���ݳ��ȣ�4*7+2*3=38
*/
void NM_swj(void)
{
	uint8_t _cnt=0;
	
	
	testdatatosend[_cnt++]=0xAA;
	testdatatosend[_cnt++]=0xFF;
	testdatatosend[_cnt++]=0xF1;
	testdatatosend[_cnt++]=34;
	if(1)
	{
			#if 0//��������������  YAW PITCH
	p=0;
			send_d_32[p++]=DJIC_IMU.total_yaw*1000;//��ǰ�Ƕ�		1
			send_d_32[p++]=DJIC_IMU.total_pitch*1000;//����Ŀ��Ƕ�		2

			send_d_32[p++]=0;//�Ӿ�����		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= PITCH_trage_angle*-50000;//I_OUT 4		4PID_YES

			send_d_32[p++]=VisionData.RawData.Depth*10;//P_OUT		5
			send_d_32[p++]=DJIC_IMU.total_pitch*-50000;//I_OUT		6
			send_d_32[p++]=VisionData.RawData.Beat*1111;//D_OUT  	7
	p=0;
			send_d_16[p++]=this_period_has_shoot_number;//�����ѹ      8

			send_d_16[p++]=yaw_trage_speed*100000;//Ŀ��Ƕ�       	9
			send_d_16[p++]=cali_sensor[i].cali_cmd*1111;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
		#if 0//����DR16����
	p=0;
			send_d_32[p++]=yaw_trage_speed*1000;//Ŀ��Ƕ�		1
			send_d_32[p++]=PITCH_MAX_angle*1000;//��ǰ�Ƕ�		2

			send_d_32[p++]=PITCH_trage_angle*1000;//P_OUT		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= PITCH_MIN_angle*1000;//I_OUT 4		4PID_YES
			send_d_32[p++]=allow_angle*1000;//P_OUT		5
			send_d_32[p++]=DR16.rc.ch1;//I_OUT		6
			send_d_32[p++]=DR16.rc.ch2;//D_OUT  	7
	p=0;
			send_d_16[p++]=DR16.rc.ch3;//�����ѹ      8

			send_d_16[p++]=0;//Ŀ��Ƕ�       	9
			send_d_16[p++]=0;//�����ѹ		10
														//������С�������λ
#endif	

		#if 0//����DR16����
	p=0;
			send_d_32[p++]=CH0_TOTAL;//Ŀ��Ƕ�		1
			send_d_32[p++]=CH1_TOTAL;//��ǰ�Ƕ�		2

			send_d_32[p++]=CH2_TOTAL;//P_OUT		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= CH3_TOTAL;//I_OUT 4		4PID_YES
			send_d_32[p++]=DR16.rc.ch0;//P_OUT		5
			send_d_32[p++]=DR16.rc.ch1;//I_OUT		6
			send_d_32[p++]=DR16.rc.ch2;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//�����ѹ      8

			send_d_16[p++]=0;//Ŀ��Ƕ�       	9
			send_d_16[p++]=0;//�����ѹ		10
														//������С�������λ
#endif
		#if 0//ȷ��������
	//����,������ʱ����ת��̨,���������ǵ�ֵ�������Ǽ�С   ��Ծ�������л��������ӵĻ�
	//���ֵ��Ŀ��ֵ����ǰֵ,��Ծ�����ֵӦΪ��
	//������ʱ����ת��̨,�����ٶ�ֵ�Ƿ�Ϊ��
	//�������:�ٶ�ֵΪ��,��˵��PID�����в���Ҫ���,���ǶȻ���Kp KiΪ��
	//Ϊ��,��˵��PID��������Ҫ���,���ǶȻ���Kp KiΪ��
	//......
	//
	//�����������
	//
	//�۲⵽����ʵ������(����PID�����):
	//�����ǵĽǶ�ֵ   �����ǵ��ٶ�ֵ  �������ֵ����������������
	//
	//�����ͬһ����ת�� �����ǵĽǶ�ֵ����,ͬʱ�����ǵ��ٶ�ֵҲΪ��(������)
	//��ô˵�������ֵΪ����ʱ��             ��Ҫ�����Ŀ���ٶ�ҲӦ��Ϊ��(������)
	//                                		                ���ǶȻ�Kp,Ki,KdӦΪ��
	//
	//��֮                                       �����ǵ��ٶ�ֵΪ��(������)
	//˵�������ֵΪ����ʱ��,              ��Ҫ���һ�������ٶ�Ŀ��ֵ(������)
	//                                		                ���ǶȻ�Kp,Ki,KdӦΪ��
	//��������,��һ����1000~3000�ĵ���ֵ
	//��������ǵ��ٶ���ʾΪ��,���ٶȻ�Kp,Ki,KdӦΪ��
	//��������ǵ��ٶ���ʾΪ��,���ٶȻ�Kp,Ki,KdӦΪ��
	
	p=0;
			send_d_32[p++]=DJIC_IMU.total_yaw*1000;//Ŀ��Ƕ�		1
			send_d_32[p++]=yaw_trage_angle*1000;//��ǰ�Ƕ�		2
	
	//�������¿�����ʱ��,������ֵ����,���ֵΪ��:Ŀ��-��ǰ
	
	//�������¿�����ʱ����ת,�������ٶ�ֵΪ��,
	
	
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//P_OUT		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= Yaw_IMU_Angle_pid.result;//I_OUT 4		4PID_YES

			send_d_32[p++]=Yaw_IMU_Speed_pid.Measure;//P_OUT		5
			send_d_32[p++]=0;//I_OUT		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//�����ѹ      8

			send_d_16[p++]=0;//Ŀ��Ƕ�       	9
			send_d_16[p++]=0;//�����ѹ		10
														//������С�������λ
#endif
	#if 0//������̨����  YAW �Ӿ�
	p=0;
			send_d_32[p++]=DJIC_IMU.total_yaw*1000;//��ǰ�Ƕ�		1
			send_d_32[p++]=yaw_trage_angle*1000;//����Ŀ��Ƕ�		2

			send_d_32[p++]=Vision_RawData_Yaw_Angle*1000;//�Ӿ�����		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= VISION_Yaw_IMU_Speed_pid.Target*10000;//I_OUT 4		4PID_YES

			send_d_32[p++]=VISION_Yaw_IMU_Speed_pid.Measure*10000;//P_OUT		5
			send_d_32[p++]=DJIC_IMU.total_pitch*-50000;//I_OUT		6
			send_d_32[p++]=VisionData.RawData.Beat*1111;//D_OUT  	7
	p=0;
			send_d_16[p++]=this_period_has_shoot_number;//�����ѹ      8

			send_d_16[p++]=yaw_trage_speed*100000;//Ŀ��Ƕ�       	9
			send_d_16[p++]=send_to_yaw*1111;//�����ѹ		10
														//������С�������λ558 320 660   bjTlta
#endif
	#if 0//�����������¶����� YAW ������
	p=0;
			send_d_32[p++]=Yaw_IMU_Speed_pid.Target*10000;//��ǰ�Ƕ�		1
			send_d_32[p++]=DJIC_IMU.total_yaw*10000;//����Ŀ��Ƕ�		2

			send_d_32[p++]=Yaw_IMU_Speed_pid.Measure*10000;//�Ӿ�����		333333333333 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch  TEMPERATURE_is_OK
			send_d_32[p++]= Yaw_IMU_Speed_pid.Error*1000;//I_OUT 4		4PID_YES

			send_d_32[p++]=Vision_RawData_Yaw_Angle*10000;//P_OUT		5
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//I_OUT	666666666666
			send_d_32[p++]=Yaw_IMU_Angle_pid.result;//D_OUT  	7 �ǶȻ������ֵ,����ľ�и���
	p=0;
			send_d_16[p++]=TEMPERATURE_PID_OUT;//�����ѹ      8

			send_d_16[p++]=bmi088_real_data.temp*10;//Ŀ��Ƕ�       	9
			send_d_16[p++]=send_to_yaw;//�����ѹ		10
														//������С�������λ558 320 660   bjTlta
#endif
#if USE_MOTOR_angle==0  //PITCHʹ�õ���Ƕ�

//������̨���� YAW ������ 666
	p=0;
			send_d_32[p++]=PITCH_trage_angle_motor;//Ŀ��Ƕ�		1
			send_d_32[p++]=GM6020s[3].totalAngle;//��ǰ�Ƕ�		2

			send_d_32[p++]=DJIC_IMU.total_pitch*1000000;//�Ӿ�����		333333333333 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch  TEMPERATURE_is_OK
			send_d_32[p++]= send_to_pitch;//�� 4		4PID_YES

			send_d_32[p++]=GM6020s[3].realCurrent;//P_OUT		5
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//I_OUT	666666666666
			send_d_32[p++]=Vision_RawData_Pitch_Angle*10000;//D_OUT  	7 �ǶȻ������ֵ,����ľ�и���
	p=0;
			send_d_16[p++]=DR16.rc.s_left;//�����ѹ      8

			send_d_16[p++]=PITCH_TRAGET_ANGLE_TEMP_EM;///*���� �Ƕ�������� �Ӿ�����ָ�������� ���ڹ��ĩ�� ��������ȫ������*/       	9
			send_d_16[p++]=cloud_mode.control_mode_NOW*111111;//�����ѹ		10
														//������С�������λ558 320 660   bjTlta
#endif

	#if 0//������̨����  PITCH ������
//��ȷ���ĸ�����PITCH��(��������������)
//��֤ˮƽʱֵΪ0������Ϊ����̧ͷΪ��
//
//Ȼ���ȷ�����»�е��λ,����Ƕ�GM6020��ֵ�������Ժ����̬��λ����
//���ֶ�һ���������߽�ֵ�Ƿ���ȷ
//Ȼ��Ϳ�����ʽ��ʼ�����ˣ��Ȱ�Ŀ��Ƕ���Ϊ0
//
//���ϵ�ʱ���ٶ�Ϊ��,������0��!
//�ǶȻ�����Ϊ��
//��ֵ��ת,
//�ٶȻ�����Ϊ��
p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//Ŀ��Ƕ�		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//��ǰ�Ƕ�		2
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//Ŀ��Ƕ�		1
			send_d_32[p++]=PITCH_IMU_Angle_pid.Measure*10000;//��ǰ�Ƕ�		2

			send_d_32[p++]=PITCH_IMU_Angle_pid.Error*10000;//P_OUT		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= Vision_RawData_Pitch_Angle*-10000;//I_OUT 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//�����ѹ      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//Ŀ���ٶ�     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//��ǰ�ٶ�		10
			send_d_32[p++]=PITCH_IMU_Speed_pid.Target*10000;//P_OUT		5
			send_d_32[p++]=PITCH_IMU_Speed_pid.Measure*10000;//I_OUT		6
			send_d_32[p++]=PITCH_IMU_Speed_pid.Differential;//D_OUT  	7
	p=0;
			send_d_16[p++]=PITCH_IMU_Speed_pid.result;//�����ѹ      8

			send_d_16[p++]=PITCH_IMU_Angle_pid.Target;//Ŀ��Ƕ�       	9
			send_d_16[p++]=send_to_pitch;//��ǰ�Ƕ�		10
														//������С�������λ
#endif
	#if 0//������̨����  ���Ӿ� ˫�� ������
//��ȷ���ĸ�����PITCH��(��������������)
//��֤ˮƽʱֵΪ0������Ϊ����̧ͷΪ��
//
//Ȼ���ȷ�����»�е��λ,����Ƕ�GM6020��ֵ�������Ժ����̬��λ����
//���ֶ�һ���������߽�ֵ�Ƿ���ȷ
//Ȼ��Ϳ�����ʽ��ʼ�����ˣ��Ȱ�Ŀ��Ƕ���Ϊ0
//
//���ϵ�ʱ���ٶ�Ϊ��,������0��!
//�ǶȻ�����Ϊ��
//��ֵ��ת,
//�ٶȻ�����Ϊ��
p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//Ŀ��Ƕ�		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//��ǰ�Ƕ�		2
			send_d_32[p++]=DJIC_IMU.total_pitch*10000;//Ŀ��Ƕ�		1
			send_d_32[p++]=PITCH_trage_angle*10000;//��ǰ�Ƕ�		2

			send_d_32[p++]=PITCH_IMU_Angle_pid.Error*10000;//P_OUT		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= Vision_RawData_Pitch_Angle*-10000;//I_OUT 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//�����ѹ      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//Ŀ���ٶ�     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//��ǰ�ٶ�		10
			send_d_32[p++]=DJIC_IMU.pitch*10000;//P_OUT		5
			send_d_32[p++]=Vision_RawData_Yaw_Angle*1000;//I_OUT		6
			send_d_32[p++]=Vision_RawData_Yaw_Angle*1000;//D_OUT  	7
	p=0;
			send_d_16[p++]=PITCH_IMU_Speed_pid.result;//�����ѹ      8

			send_d_16[p++]=PITCH_IMU_Angle_pid.Target;//Ŀ��Ƕ�       	9
			send_d_16[p++]=send_to_pitch;//��ǰ�Ƕ�		10
														//������С�������λ
#endif
#if 0//���͵���3508����
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//Ŀ��Ƕ�		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//��ǰ�Ƕ�		2
			send_d_32[p++]=CHASSIS_trage_angle;//Ŀ��Ƕ�		1
			send_d_32[p++]=M3508s[3].totalAngle;//��ǰ�Ƕ�		2

			send_d_32[p++]=CHASSIS_MOTOR_SPEED_pid.Target;//Ŀ���ٶ�		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= CHASSIS_MOTOR_SPEED_pid.Measure;//��ǰ�ٶ� 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//�����ѹ      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//Ŀ���ٶ�     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//��ǰ�ٶ�		10
			send_d_32[p++]=CHASSIS_MOTOR_SPEED_pid.Proportion;//P_OUT		5
			send_d_32[p++]=CHASSIS_MOTOR_SPEED_pid.I_Output;//I_OUT		6
			send_d_32[p++]=CHASSIS_MOTOR_SPEED_pid.D_Output;//D_OUT  	7
	p=0;
			send_d_16[p++]=CHASSIS_MOTOR_SPEED_pid.result;//�����ѹ      8

			send_d_16[p++]=0;//Ŀ��Ƕ�       	9
			send_d_16[p++]=0;//��ǰ�Ƕ�		10

#endif
#if 0//����Ħ����3508����
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//Ŀ��Ƕ�		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//��ǰ�Ƕ�		2
			send_d_32[p++]=6700;//Ŀ��Ƕ�		1
			send_d_32[p++]=-M3508s[2].realSpeed;//��ǰ�Ƕ�		2

			send_d_32[p++]=0;//Ŀ���ٶ�		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= M3508s[3].realSpeed;//��Ħ���� 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//�����ѹ      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//Ŀ���ٶ�     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//��ǰ�ٶ�		10
			send_d_32[p++]=0;//P_OUT		5
			send_d_32[p++]=0;//I_OUT		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//�����ѹ      8

			send_d_16[p++]=0;//Ŀ��Ƕ�       	9
			send_d_16[p++]=0;//��ǰ�Ƕ�		10

#endif
#if 0//���͹������
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//Ŀ��Ƕ�		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//��ǰ�Ƕ�		2
			send_d_32[p++]=M3508s[3].realSpeed;//Ŀ��Ƕ�		1
			send_d_32[p++]=HWswitch_R;//��ǰ�Ƕ�		2

			send_d_32[p++]=HWswitch_L;//Ŀ���ٶ�		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= M3508s[3].totalAngle;//��ǰ�ٶ� 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//�����ѹ      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//Ŀ���ٶ�     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//��ǰ�ٶ�		10
			send_d_32[p++]=0;//P_OUT		5
			send_d_32[p++]=0;//I_OUT		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//�����ѹ      8

			send_d_16[p++]=0;//Ŀ��Ƕ�       	9
			send_d_16[p++]=0;//��ǰ�Ƕ�		10

#endif


#if 0//���͵���λ������
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//Ŀ��Ƕ�		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//��ǰ�Ƕ�		2
			send_d_32[p++]=CHASSIS_trage_angle;//Ŀ��Ƕ�		1
			send_d_32[p++]=CHASSIS_L_MAX;//��ǰ�Ƕ�		2

			send_d_32[p++]=M3508s[3].totalAngle;//Ŀ���ٶ�		3 
			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= CHASSIS_R_MIN;//��ǰ�ٶ� 4		4PID_YES
//			send_d_32[p++]=Yaw_Angle_pid.Integral;//I_OUT 4		4
//			send_d_32[4]=Yaw_Angle_pid.Differential;//D_OUT		

//			send_d_32[p++]=Yaw_Speed_pid.Proportion;//P_OUT		5
//			send_d_32[p++]=Yaw_Speed_pid.I_Output;//I_OUT		6
//			send_d_32[p++]=Yaw_Speed_pid.Differential;//D_OUT  	7
//	p=0;
//			send_d_16[p++]=Yaw_Speed_pid.result;//�����ѹ      8

//			send_d_16[p++]=Yaw_Speed_pid.Target;//Ŀ���ٶ�     	9
//			send_d_16[p++]=Yaw_Speed_pid.Measure;//��ǰ�ٶ�		10
			send_d_32[p++]=CHASSIS_MID;//P_OUT		5
			send_d_32[p++]=DEBUFF*100;//I_OUT		6
			send_d_32[p++]=speed_change;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//�����ѹ      8

			send_d_16[p++]=M3508s[3].realSpeed;//Ŀ��Ƕ�       	9
			send_d_16[p++]=RANDOM_CHASSIS.number;//�����		10

#endif

#if 0//���͵���λ������
	p=0;
			ENCODER_M_MID=(ENCODER_L_MAX+ENCODER_R_MIN)/2;
			send_d_32[p++]=CHASSIS_L_MAX;//�����߽�ֵ		1
			send_d_32[p++]=M3508s[3].totalAngle;//�ڹ�λ��		2

			send_d_32[p++]=CHASSIS_R_MIN;//����ұ߽�ֵ		3 

			send_d_32[p++]=ENCODER_L_MAX;//��ǰ�ٶ� 4		4PID_YES

			send_d_32[p++]=Chassis_Encoder.totalLine;//P_OUT		5
			send_d_32[p++]=ENCODER_R_MIN;//I_OUT		6
			send_d_32[p++]=ENCODER_M_MID;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//�����ѹ      8

			send_d_16[p++]=0;//Ŀ��Ƕ�       	9
			send_d_16[p++]=0;//�����		10

#endif

#if 0//���ͱ�����λ������
	p=0;
			ENCODER_M_MID=(ENCODER_L_MAX+ENCODER_R_MIN)/2;
			send_d_32[p++]=ENCODER_ARRIVE_MAX;//�����߽�ֵ		1
			send_d_32[p++]=Chassis_Encoder.totalLine;//�ڹ�λ��		2

			send_d_32[p++]=ENCODER_ARRIVE_MIN;//����ұ߽�ֵ		3 

			send_d_32[p++]=ENCODER_L_MAX;//��ǰ�ٶ� 4		4PID_YES

			send_d_32[p++]=Chassis_Encoder.totalLine;//P_OUT		5
			send_d_32[p++]=ENCODER_R_MIN;//I_OUT		6
			send_d_32[p++]=ENCODER_M_MID;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//�����ѹ      8

			send_d_16[p++]=0;//Ŀ��Ƕ�       	9
			send_d_16[p++]=speed_change_times;//�����		10

#endif

#if 0//���Ͳ�������
	p=0;
			send_d_32[p++]=Driver_I_PID.Target;//Ŀ���ٶ�		1
			send_d_32[p++]=M3508s[1].realSpeed;//��ǰ�ٶ�		2

			send_d_32[p++]=M3508s[1].totalAngle;//����ұ߽�ֵ		3 

			send_d_32[p++]=M3508s[1].realAngle;//��ǰ�ٶ� 4		4PID_YES

			send_d_32[p++]=0;//P_OUT		5
			send_d_32[p++]=0;//I_OUT		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=Driver_I_PID.Proportion;//�����ѹ      8

			send_d_16[p++]=Driver_I_PID.Differential;//Ŀ��Ƕ�       	9
			send_d_16[p++]=Driver_I_PID.result;//�����		10

#endif

#if 0//���Ͳ�������
send_data10=M3508s[2].realSpeed;
	p=0;
			send_d_32[p++]=0;//Ŀ���ٶ�		1
			send_d_32[p++]=send_data10;//��ǰ�ٶ�		2

			send_d_32[p++]=0;//����ұ߽�ֵ		3 

			send_d_32[p++]=0;//��ǰ�ٶ� 4		4PID_YES

			send_d_32[p++]=0;//P_OUT		5
			send_d_32[p++]=0;//I_OUT		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//�����ѹ      8

			send_d_16[p++]=0;//Ŀ��Ƕ�       	9
			send_d_16[p++]=0;//�����		10

#endif
}
if(1)
{
	#if 1//�����Զ���������  ��
	p=0;

			send_d_32[p++]=M3508s[1].totalAngle;//���̵�ǰ�Ƕ�		1
			send_d_32[p++]=Driver_ANGLE_pid.Target;//����Ŀ��Ƕ�		2

			send_d_32[p++]=DJIC_IMU.total_yaw*10000;//����Ŀ��Ƕ�		2
	
			send_d_32[p++]= -M3508s[3].realSpeed;//��Ħ���� 4		4PID_YES
			send_d_32[p++]=ext_shoot_data.data.bullet_speed*10;//����Ŀ��Ƕ�		5
			send_d_32[p++]=ext_power_heat_data.data.shooter_id2_17mm_cooling_heat;//���̵�ǰ�Ƕ�		6
			send_d_32[p++]=ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;//�Ӿ�����ָ��  	7
	p=0;
	
			send_d_16[p++]=vision_shoot_times;//���    ָ�����      8
			send_d_16[p++]=SHOOT_STOP_time;//ֹͣ���   ָ�����       	9
			send_d_16[p++]=ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;//ǹ��1����		10

#endif
#if 0//����Ħ��������  ��
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//Ŀ��Ƕ�		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//��ǰ�Ƕ�		2
			send_d_32[p++]=SHOOT_L_speed;//Ŀ��Ƕ�		1
			send_d_32[p++]=M3508s[2].realSpeed;//��ǰ�Ƕ�		2

			send_d_32[p++]=-SHOOT_R_speed;//Ŀ���ٶ�		3 
			send_d_32[p++]= -M3508s[3].realSpeed;//��Ħ���� 4		4PID_YES
			send_d_32[p++]=Driver_ANGLE_pid.Target;//����Ŀ��Ƕ�		5
			send_d_32[p++]=ext_game_robot_state.data.shooter_id1_17mm_cooling_rate;//���̵�ǰ�Ƕ�		6
			send_d_32[p++]=ext_shoot_data.data.bullet_speed*100;//D_OUT  	7
	p=0;
			send_d_16[p++]=JS_RC_TIMES;//�����ѹ      8
			send_d_16[p++]=send_to_SHOOT_L;//Ŀ��Ƕ�       	9
			send_d_16[p++]=ext_power_heat_data.data.shooter_id1_17mm_cooling_heat;//��ǰ�Ƕ�		10

#endif
#if 0//������Ħ��������  
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//Ŀ��Ƕ�		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//��ǰ�Ƕ�		2
			send_d_32[p++]=SHOOT_L_I_PID.Proportion;//Ŀ���ٶ�		1
			send_d_32[p++]=SHOOT_L_I_PID.Increment_Output;//��ǰ�ٶ�		2

			send_d_32[p++]=SHOOT_L_I_PID.D_Output;//�ٶ����		3 
			send_d_32[p++]= SHOOT_R_I_PID.Proportion;//�����ʱ,����������� 4		4PID_YES
			send_d_32[p++]=SHOOT_R_I_PID.Increment_Output;//����Ŀ��Ƕ�		5
			send_d_32[p++]=SHOOT_R_I_PID.D_Output;//���̵�ǰ�Ƕ�		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=send_to_SHOOT_L;//�����ѹ      8
			send_d_16[p++]=SHOOT_L_I_PID.Target;//Ŀ��Ƕ�       	9
			send_d_16[p++]=SHOOT_L_I_PID.Measure;//��ǰ�Ƕ�		10

#endif
#if 0//������Ħ��������  
	p=0;

//			send_d_32[p++]=Yaw_Angle_pid.Target;//Ŀ��Ƕ�		1
//			send_d_32[p++]=Yaw_Angle_pid.Measure;//��ǰ�Ƕ�		2
			send_d_32[p++]=6700;//Ŀ���ٶ�		1
			send_d_32[p++]=SHOOT_L_I_PID.Measure;//��ǰ�ٶ�		2

			send_d_32[p++]=SHOOT_L_I_PID.Error;//�ٶ����		3 
			send_d_32[p++]= send_to_SHOOT_L;//�����ʱ,����������� 4		4PID_YES
			send_d_32[p++]=0;//����Ŀ��Ƕ�		5
			send_d_32[p++]=Driver_ANGLE_pid.Measure;//���̵�ǰ�Ƕ�		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=0;//�����ѹ      8
			send_d_16[p++]=0;//Ŀ��Ƕ�       	9
			send_d_16[p++]=0;//��ǰ�Ƕ�		10

#endif

	#if 0//���Ϳ��������� YAW ������ 666
	p=0;
			send_d_32[p++]=PITCH_Angle_pid.Error*10000;//��ǰ�Ƕ�		1
			send_d_32[p++]=DJIC_IMU.total_pitch*10000;//����Ŀ��Ƕ�		2

			send_d_32[p++]=Vision_RawData_Pitch_Angle*10000;//�Ӿ�ԭʼ����		333333333333 

			send_d_32[p++]= PITCH_TRAGET_ANGLE_TEMP*10000;// 4 ֮ǰ�����ֵ��Ŀ��ֵ

			send_d_32[p++]=PITCH_IMU_Speed_pid.P_Output*10000;//P_OUT		5
			send_d_32[p++]=PITCH_IMU_Speed_pid.I_Output*10000;//I_OUT	666666666666
			send_d_32[p++]=PITCH_IMU_Speed_pid.D_Output*10000;//D_OUT  	7 �ǶȻ������ֵ,����ľ�и���
	p=0;
			send_d_16[p++]=PITCH_IMU_Speed_pid.result;//�����ѹ      8

			send_d_16[p++]=auto_shoot_condition_show;///*���� �Ƕ�������� �Ӿ�����ָ�������� ���ڹ��ĩ�� ��������ȫ������*/       	9
			send_d_16[p++]=cloud_mode.control_mode_NOW*111111;//�����ѹ		10
														//������С�������λ558 320 660   bjTlta
#endif
	#if 0//���Ϳ��������� YAW ������ 666
	p=0;
			send_d_32[p++]=VISION_Yaw_IMU_Angle_pid.Error*10000;//��ǰ�Ƕ�		1
			send_d_32[p++]=DJIC_IMU.total_pitch*10000;//����Ŀ��Ƕ�		2

			send_d_32[p++]=Vision_RawData_Pitch_Angle*10000;//�Ӿ�ԭʼ����		333333333333 

			send_d_32[p++]= PITCH_TRAGET_ANGLE_TEMP*10000;// 4 ֮ǰ�����ֵ��Ŀ��ֵ

			send_d_32[p++]=PITCH_IMU_Speed_pid.P_Output*10000;//P_OUT		5
			send_d_32[p++]=PITCH_IMU_Speed_pid.I_Output*10000;//I_OUT	666666666666
			send_d_32[p++]=PITCH_IMU_Speed_pid.D_Output*10000;//D_OUT  	7 �ǶȻ������ֵ,����ľ�и���
	p=0;
			send_d_16[p++]=PITCH_IMU_Speed_pid.result;//�����ѹ      8

			send_d_16[p++]=auto_shoot_condition_show;///*���� �Ƕ�������� �Ӿ�����ָ�������� ���ڹ��ĩ�� ��������ȫ������*/       	9
			send_d_16[p++]=cloud_mode.control_mode_NOW*111111;//�����ѹ		10
														//������С�������λ558 320 660   bjTlta
#endif

#if 0//�Ӿ�����ʱҪ����

	p=0;
			send_d_32[p++]=PITCH_trage_angle_motor;//PITCH Ŀ��Ƕ�		1
			send_d_32[p++]=GM6020s[3].totalAngle;//PITCH   ��ǰ�Ƕ�		2

			send_d_32[p++]=VISION_Yaw_IMU_Angle_pid.Error*10000;//YAW �Ƕ����		333333333333 

			send_d_32[p++]= VISION_Yaw_IMU_Speed_pid.Target*100;//YAW �ٶ�Ŀ�� 4		4PID_YES

			send_d_32[p++]=VISION_Yaw_IMU_Speed_pid.Error*100;//YAW �ٶ����           P_OUT		5
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//I_OUT	666666666666
			send_d_32[p++]=Vision_RawData_Pitch_Angle*10000;//D_OUT  	7 �ǶȻ������ֵ,����ľ�и���
	p=0;
			send_d_16[p++]=VisionData.RawData.Beat*10+VisionData.RawData.Armour;//����,�Զ�����      8

			send_d_16[p++]=PITCH_TRAGET_ANGLE_TEMP_EM;///*���� �Ƕ�������� �Ӿ�����ָ�������� ���ڹ��ĩ�� ��������ȫ������*/       	9
			send_d_16[p++]=cloud_mode.control_mode_NOW*111111;//�����ѹ		10
														//������С�������λ558 320 660   bjTlta

#endif
#if 0//���Ӿ�YAWʱҪ����

	p=0;
			send_d_32[p++]=PITCH_trage_angle_motor;//Ŀ��Ƕ�		1
			send_d_32[p++]=GM6020s[3].totalAngle;//��ǰ�Ƕ�		2

			send_d_32[p++]=VISION_Yaw_IMU_Angle_pid.Error*10000;//�Ӿ�����		333333333333 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch  TEMPERATURE_is_OK
			send_d_32[p++]= VISION_Yaw_IMU_Speed_pid.Target*100;//�� 4		4PID_YES

			send_d_32[p++]=VISION_Yaw_IMU_Speed_pid.Error*100;//P_OUT		5
			send_d_32[p++]=Yaw_IMU_Angle_pid.Error*10000;//I_OUT	666666666666
			send_d_32[p++]=Vision_RawData_Pitch_Angle*10000;//D_OUT  	7 �ǶȻ������ֵ,����ľ�и���
	p=0;
			send_d_16[p++]=VisionData.RawData.Beat*10+VisionData.RawData.Armour;//�����ѹ      8

			send_d_16[p++]=PITCH_TRAGET_ANGLE_TEMP_EM;///*���� �Ƕ�������� �Ӿ�����ָ�������� ���ڹ��ĩ�� ��������ȫ������*/       	9
			send_d_16[p++]=cloud_mode.control_mode_NOW*111111;//�����ѹ		10
														//������С�������λ558 320 660   bjTlta

#endif
}
	


for(p=0;p<7;p++)
	{
testdatatosend[_cnt++]=BYTE0(send_d_32[p]);
testdatatosend[_cnt++]=BYTE1(send_d_32[p]);
testdatatosend[_cnt++]=BYTE2(send_d_32[p]);
testdatatosend[_cnt++]=BYTE3(send_d_32[p]);
	}	
	for(p=0;p<3;p++)
	{
testdatatosend[_cnt++]=BYTE0(send_d_16[p]);
testdatatosend[_cnt++]=BYTE1(send_d_16[p]);
	}
	
	
sumcheck=0;
addcheck=0;
for(i=0; i < (testdatatosend[3]+4); i++)
{
sumcheck += testdatatosend[i]; //��֡ͷ��ʼ����ÿһ�ֽڽ�����ͣ�ֱ��DATA������
addcheck += sumcheck; //ÿһ�ֽڵ���Ͳ���������һ��sumcheck���ۼ�
}
	testdatatosend[_cnt++]=sumcheck;	
	testdatatosend[_cnt++]=addcheck;	

	HAL_UART_Transmit_DMA(&huart1,&testdatatosend[0],_cnt);//4pin

//	CDC_Transmit_FS(&testdatatosend[0],_cnt);

//		for (uint8_t i = 0; i < _cnt; i++)
//	{
//		while ((USART6->SR & 0X40) == 0);
//		USART6->DR = testdatatosend[i];
//	}//��PIN�ӿ�
}


void NM_swj2(void)
{
	uint8_t _cnt=0;
	
	
	testdatatosend[_cnt++]=0xAA;
	testdatatosend[_cnt++]=0xFF;
	testdatatosend[_cnt++]=0xF2;
	testdatatosend[_cnt++]=10;
//	testdatatosend[_cnt++]=0;
//	
//	testdatatosend[_cnt++]=BYTE0(mubiaosudu3);
//			testdatatosend[_cnt++]=BYTE1(mubiaosudu3);

//	testdatatosend[_cnt++]=BYTE0(my_6020array[1].realSpeed);
//		testdatatosend[_cnt++]=BYTE1(my_6020array[1].realSpeed);
	//λ�û�����
//	data3=GM_6020_speed.p_out;
//	data4=GM_6020_speed.i_out;
	
//testdatatosend[_cnt++]=BYTE0(targe_angle);
//testdatatosend[_cnt++]=BYTE1(targe_angle);

testdatatosend[_cnt++]=BYTE0(data1);
testdatatosend[_cnt++]=BYTE1(data1);//��ʵ�ٶ�

testdatatosend[_cnt++]=BYTE0(data3);
testdatatosend[_cnt++]=BYTE1(data3);
testdatatosend[_cnt++]=BYTE2(data3);
testdatatosend[_cnt++]=BYTE3(data3);//speed p����out

testdatatosend[_cnt++]=BYTE0(data4);
testdatatosend[_cnt++]=BYTE1(data4);
testdatatosend[_cnt++]=BYTE2(data4);
testdatatosend[_cnt++]=BYTE3(data4);//speed i����out
//testdatatosend[_cnt++]=BYTE0(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE1(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE2(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE3(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE0(mubiaosudu3);
//testdatatosend[_cnt++]=BYTE1(mubiaosudu3);//���



//	testdatatosend[_cnt++]=BYTE0(data3);
//testdatatosend[_cnt++]=BYTE1(data3);
//testdatatosend[_cnt++]=BYTE2(data3);
//testdatatosend[_cnt++]=BYTE3(data3);//Ŀ��Ƕ�
//testdatatosend[_cnt++]=BYTE0(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE1(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE2(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE3(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE0(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE1(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE2(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE3(GM_6020_speed.d_out);
//	testdatatosend[_cnt++]=BYTE3(mubiaosudu3);
//	

sumcheck=0;
addcheck=0;
for(i=0; i < (testdatatosend[3]+4); i++)
{
sumcheck += testdatatosend[i]; //��֡ͷ��ʼ����ÿһ�ֽڽ�����ͣ�ֱ��DATA������
addcheck += sumcheck; //ÿһ�ֽڵ���Ͳ���������һ��sumcheck���ۼ�
}
	testdatatosend[_cnt++]=sumcheck;	
	testdatatosend[_cnt++]=addcheck;	

//	HAL_UART_Transmit_DMA(&huart6,&testdatatosend[0],_cnt);


}





