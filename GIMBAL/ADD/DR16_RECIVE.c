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
#include "MY_balance_CONTROL.h"
#include "MIT.h"

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
int16_t send_d_16[3];//+-32767

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
				#if 0//ƽ�� λ�û�ͨ�õ���
		
	p=0;
			send_d_32[p++]=BALANCE_P.result;//����ֵĿ��λ��		1
			send_d_32[p++]=BALANCE_P.result+BALANCE_I.result;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=send_to_tire_L;//�Ƕ��� Ŀ��λ��		3 
		
			send_d_32[p++]= DJIC_IMU.Gyro_y*100;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MIT_B.ANGLE_JD-MIT_B.MIT_TZG;////Ŀ���ٶ�-������		5
		
			send_d_32[p++]=TARGET_position_V2;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=milemeter_test.total_mile_truly_use;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=TARGET_angle_PITCH_BC*100;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=DJIC_IMU.total_pitch*100;//ʵ��       	9
			send_d_16[p++]=liftoff_R*10;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif
		#if 0//ƽ�� ֱ����ͨ�õ���
		
	p=0;
			send_d_32[p++]=BALANCE_P.result;//����ֵĿ��λ��		1
			send_d_32[p++]=BALANCE_P.result+BALANCE_I.result;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=send_to_tire_L;//�Ƕ��� Ŀ��λ��		3 
		
			send_d_32[p++]= DJIC_IMU.Gyro_y*100;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MIT_B.ANGLE_JD-MIT_B.MIT_TZG;////Ŀ���ٶ�-������		5
		
			send_d_32[p++]=DJIC_IMU.total_pitch*1000;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=L_speed_new-R_speed_new*10;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=TARGET_angle_PITCH_BC*100;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=DJIC_IMU.total_pitch*100;//ʵ��       	9
			send_d_16[p++]=liftoff_R*10;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
		#if 0//ƽ�� ���PID��������  �ؽ� �ȵ���б�Ƕ�
		
	p=0;
			send_d_32[p++]=POSITION_v2.result*1000;//����ֵĿ��λ��		1
			send_d_32[p++]=POSITION_v2.result*PITCH_XR_K*1000;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=MIT_B.RC_TIMES;//�Ƕ��� Ŀ��λ��		3 
		
			send_d_32[p++]= MIT_A.ANGLE_JD-MIT_A.MIT_TZG;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MIT_B.ANGLE_JD-MIT_B.MIT_TZG;////Ŀ���ٶ�-������		5
		
			send_d_32[p++]=((MIT_A.ANGLE_JD-MIT_A.MIT_TZG)-(MIT_B.MIT_TZG-MIT_B.ANGLE_JD))/2*10;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=MIT_Bias_R*10;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=MIT_D.TX_TIMES;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=MIT_B.MIT_TSZ*10;//ʵ��       	9
			send_d_16[p++]=liftoff_R*10;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
							#if 0//MIT ABCD ������ն�֡���
		
	p=0;
			send_d_32[p++]=MIT_A.RC_TIMES;//����ֵĿ��λ��		1
			send_d_32[p++]=MIT_A.TX_TIMES;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=MIT_B.RC_TIMES;//�Ƕ��� Ŀ��λ��		3 
		
			send_d_32[p++]= MIT_B.TX_TIMES;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MIT_C.RC_TIMES;////Ŀ���ٶ�-������		5
		
			send_d_32[p++]=MIT_C.TX_TIMES;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=MIT_D.RC_TIMES;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=MIT_D.TX_TIMES;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=MIT_B.MIT_TSZ*10;//ʵ��       	9
			send_d_16[p++]=liftoff_R*10;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
		
							#if 0//б�º�����ô��?
		
	p=0;
			send_d_32[p++]=SEND_TO_MIT_MAX.Current_Value*10;//����ֵĿ��λ��		1
			send_d_32[p++]=SEND_TO_MIT_MAX.Target_Value*10;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=SEND_TO_MIT_MAX.Rate*1000;//�Ƕ��� Ŀ��λ��		3 
		
			send_d_32[p++]= SEND_TO_MIT_MAX.Absolute_Max*10;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MAX_OUT*10;////Ŀ���ٶ�-������		5
		
			send_d_32[p++]=MAX_OUT;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=send_to_MIT_damping*10000;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=MIT_DISABLE_TIMES	;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=MIT_ENABLE_TIMES;//ʵ��       	9
			send_d_16[p++]=liftoff_R*10;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif
											#if 1//MIT C��D ���м��
		
	p=0;
			send_d_32[p++]=MIT_C.ANGLE_JD*10;//����ֵĿ��λ��		1
			send_d_32[p++]=MIT_C.target_position*10;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=MIT_C.target_position_end*10;//�Ƕ��� Ŀ��λ��		3 
		
			send_d_32[p++]=(MIT_C.MIT_TZG+3)*10;//����ֵĿ��λ��		1
			send_d_32[p++]=MIT_D.target_position*10;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=MIT_D.position_end*10;//�Ƕ��� Ŀ��λ��		3 		
			send_d_32[p++]=target_speed_text*1000;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=MIT_B_SPEED.Max_result;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=0;//ʵ��       	9
			send_d_16[p++]=liftoff_L;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
									#if 0//MIT A��B ���м��
		
	p=0;
			send_d_32[p++]=MIT_A.ANGLE_JD*10;//����ֵĿ��λ��		1
			send_d_32[p++]=MIT_A.target_position*10;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=MIT_A.send_to_MIT*10;//�Ƕ��� Ŀ��λ��		3 
		
			send_d_32[p++]=MIT_B.ANGLE_JD*10;//����ֵĿ��λ��		1
			send_d_32[p++]=MIT_B.target_position*10;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=MIT_B.send_to_MIT*10;//�Ƕ��� Ŀ��λ��		3 		
			send_d_32[p++]=target_speed_text*1000;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=0;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=0;//ʵ��       	9
			send_d_16[p++]=liftoff_R;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
		
		
											#if 0//MIT D ���м��
		
	p=0;
			send_d_32[p++]=MIT_D.ANGLE_JD*10;//����ֵĿ��λ��		1
			send_d_32[p++]=MIT_D.target_position*10;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=MIT_D.send_to_MIT*10;//�Ƕ��� Ŀ��λ��		3 
		
			send_d_32[p++]= MIT_D_SPEED.Target*1000;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MIT_D.SPEED_JD*1000;////Ŀ���ٶ�-������		5
		
			send_d_32[p++]=text_moto.SPEED_JD*1000;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=target_speed_text*1000;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=0;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=0;//ʵ��       	9
			send_d_16[p++]=liftoff_L*10;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
									#if 0//MIT C ���м��
		
	p=0;
			send_d_32[p++]=MIT_C.ANGLE_JD*10;//����ֵĿ��λ��		1
			send_d_32[p++]=MIT_C.target_position*10;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=MIT_C.send_to_MIT*10;//�Ƕ��� Ŀ��λ��		3 
		
			send_d_32[p++]= MIT_C_SPEED.Target*1000;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MIT_C.SPEED_JD*1000;////Ŀ���ٶ�-������		5
		
			send_d_32[p++]=text_moto.SPEED_JD*1000;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=target_speed_text*1000;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=0;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=0;//ʵ��       	9
			send_d_16[p++]=liftoff_L*10;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
							#if 0//MIT A ���м��
		
	p=0;
			send_d_32[p++]=MIT_A.ANGLE_JD*10;//����ֵĿ��λ��		1
			send_d_32[p++]=MIT_A.target_position*10;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=MIT_A.send_to_MIT*10;//�Ƕ��� Ŀ��λ��		3 
		
			send_d_32[p++]= MIT_A_SPEED.Target*1000;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MIT_A.SPEED_JD*1000;////Ŀ���ٶ�-������		5
		
			send_d_32[p++]=text_moto.SPEED_JD*1000;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=target_speed_text*1000;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=MIT_A.MIT_TZG*10;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=MIT_A.MIT_TSZ*10;//ʵ��       	9
			send_d_16[p++]=liftoff_R*10;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
					#if 0//MIT B ���м��
		
	p=0;
			send_d_32[p++]=MIT_B.ANGLE_JD*10;//����ֵĿ��λ��		1
			send_d_32[p++]=MIT_B.target_position*10;//����ֵ��ǰλ��		2
		
			send_d_32[p++]=MIT_B.send_to_MIT*10;//�Ƕ��� Ŀ��λ��		3 
		
			send_d_32[p++]= MIT_B_SPEED.Target*1000;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MIT_B.SPEED_JD*1000;////Ŀ���ٶ�-������		5
		
			send_d_32[p++]=MIT_B.velocity_end*10;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=MIT_B_SPEED.Max_result;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=MIT_B.MIT_TZG*10;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=MIT_B.MIT_TSZ*10;//ʵ��       	9
			send_d_16[p++]=liftoff_R*10;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
			#if 0//MIT ����λ�û� 
		
	p=0;
			send_d_32[p++]=send_to_MIT_text*1000;//����ֵĿ��λ��		1
			send_d_32[p++]=target_position_text_PID*10;//����ֵ��ǰλ��		2
			send_d_32[p++]=text_moto.ANGLE_JD*10;//�Ƕ��� Ŀ��λ��		3 
			send_d_32[p++]= target_speed_text*1000;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MIT_SPEED_TEXT.Proportion*1000;////Ŀ���ٶ�-������		5
			send_d_32[p++]=text_moto.SPEED_JD*1000;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=target_speed_text*1000;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=0;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=0;//ʵ��       	9
			send_d_16[p++]=target_speed_text*1000;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
		
					#if 0//MIT Ѱ�����õ��ٶ�ֵ 
		
	p=0;
			send_d_32[p++]=MIT_SPEED_NEW;//����ֵĿ��λ��		1
			send_d_32[p++]=text_moto.velocity;//����ֵ��ǰλ��		2
			send_d_32[p++]=2045;//�Ƕ��� Ŀ��λ��		3 
			send_d_32[p++]= text_moto.SPEED_JD*1000;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MIT_SPEED_BY_ANGLE;////Ŀ���ٶ�-������		5
			send_d_32[p++]=text_moto.velocity;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=MIT_SPEED_NEW;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=0;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=0;//ʵ��       	9
			send_d_16[p++]=0;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
			#if 0//MIT �����ٶȻ� 
		
	p=0;
			send_d_32[p++]=send_to_MIT_text*1000;//����ֵĿ��λ��		1
			send_d_32[p++]=MIT_SPEED_BY_ANGLE;//����ֵ��ǰλ��		2
			send_d_32[p++]=position_text*1000;//�Ƕ��� Ŀ��λ��		3 
			send_d_32[p++]= MIT_SPEED_TEXT.I_Output*1000;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=MIT_SPEED_TEXT.Proportion*1000;////Ŀ���ٶ�-������		5
			send_d_32[p++]=text_moto.SPEED_JD*1000;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=target_speed_text*1000;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=0;//������Ŀ���ٶ���ֵ,����Ϊ��ֵ;//�����ѹ      8

			send_d_16[p++]=0;//ʵ��       	9
			send_d_16[p++]=target_speed_text*1000;//Ŀ��		10
														//������С�������λ558 320 660   bjTlta
#endif	
											#if 0//MIT �ٶȻ� 
		
	p=0;
			send_d_32[p++]=position_HD_text*10000;//����ֵĿ��λ��		1
			send_d_32[p++]=text_moto.position_end*10000;//����ֵ��ǰλ��		2
			send_d_32[p++]=position_text*10000;//�Ƕ��� Ŀ��λ��		3 
			send_d_32[p++]= text_moto.ANGLE_JD*10000;//�Ƕ��� ��ǰλ�� 4		4PID_YES
			send_d_32[p++]=speed_HD_text*100;////Ŀ���ٶ�-������		5
			send_d_32[p++]=text_moto.velocity_end*100;//��ǰ�ٶ�-������	6		
			send_d_32[p++]=speed_text*100;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=SPEED_P_v2.Proportion;//�����ѹ      8

			send_d_16[p++]=M3508s[3].realSpeed;//Ŀ��Ƕ�       	9
			send_d_16[p++]=send_to_tire_L;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif		
		#if 0//MIT���β���
		
	p=0;
			send_d_32[p++]=text_moto.ANGLE_JD*10000;//Ŀ��λ��		1
			send_d_32[p++]=text_moto.SPEED_JD*100;//��ǰλ��		2

			send_d_32[p++]=position_text*10000;//Ŀ����̬�Ƕ�		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= text_moto.velocity_end*100;//I_OUT 4		4PID_YES

			send_d_32[p++]=RC_SPEED_TO_POSITION.result;//P_OUT		5
		
			send_d_32[p++]=DR16.rc.ch1;//��̼Ʋ���		6
			send_d_32[p++]=speed_text*100;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=SPEED_P_v2.Proportion;//�����ѹ      8

			send_d_16[p++]=M3508s[3].realSpeed;//Ŀ��Ƕ�       	9
			send_d_16[p++]=send_to_tire_L;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
		
									#if 0//MITͨ�Ž��ղ���
		
	p=0;
			send_d_32[p++]=text_moto.ANGLE_JD*10000;//Ŀ��λ��		1
			send_d_32[p++]=text_moto.SPEED_JD*100;//��ǰλ��		2

			send_d_32[p++]=text_moto.velocity;//Ŀ����̬�Ƕ�		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= text_moto.velocity_end*100;//I_OUT 4		4PID_YES

			send_d_32[p++]=RC_SPEED_TO_POSITION.result;//P_OUT		5
		
			send_d_32[p++]=DR16.rc.ch1;//��̼Ʋ���		6
			send_d_32[p++]=L_speed_new;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=SPEED_P_v2.Proportion;//�����ѹ      8

			send_d_16[p++]=M3508s[3].realSpeed;//Ŀ��Ƕ�       	9
			send_d_16[p++]=send_to_tire_L;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
							#if 0//ǰ����ң��������
		
	p=0;
			send_d_32[p++]=POSITION_v2.Error;//Ŀ��λ��		1
			send_d_32[p++]=TARGET_speed_RC;//��ǰλ��		2

			send_d_32[p++]=L_speed_new-R_speed_new;//Ŀ����̬�Ƕ�		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= DJIC_IMU.Gyro_z;//I_OUT 4		4PID_YES

			send_d_32[p++]=RC_SPEED_TO_POSITION.result;//P_OUT		5
		
			send_d_32[p++]=DR16.rc.ch1;//��̼Ʋ���		6
			send_d_32[p++]=L_speed_new;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=SPEED_P_v2.Proportion;//�����ѹ      8

			send_d_16[p++]=M3508s[3].realSpeed;//Ŀ��Ƕ�       	9
			send_d_16[p++]=send_to_tire_L;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
		
					#if 0//YAW�����
		
	p=0;
			send_d_32[p++]=DJIC_IMU.total_yaw*10000;//Ŀ��λ��		1
			send_d_32[p++]=TARGET_angle_YAW*10000;//��ǰλ��		2

			send_d_32[p++]=TARGET_angle_speed_YAW;//Ŀ����̬�Ƕ�		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= DJIC_IMU.Gyro_z;//I_OUT 4		4PID_YES

			send_d_32[p++]=L_speed_new-R_speed_new;//P_OUT		5
		
			send_d_32[p++]=DR16.rc.ch1;//��̼Ʋ���		6
			send_d_32[p++]=L_speed_new;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=SPEED_P_v2.Proportion;//�����ѹ      8

			send_d_16[p++]=M3508s[3].realSpeed;//Ŀ��Ƕ�       	9
			send_d_16[p++]=send_to_tire_L;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
			#if 0//�˲�����
		
	p=0;
			send_d_32[p++]=M3508s[3].realSpeed;//Ŀ��λ��		1
			send_d_32[p++]=L_speed_new;//��ǰλ��		2

			send_d_32[p++]=M3508s[2].realSpeed;//Ŀ����̬�Ƕ�		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= R_speed_new;//I_OUT 4		4PID_YES

			send_d_32[p++]=milemeter_test.total_mile_by_turnCount;//P_OUT		5
		
			send_d_32[p++]=milemeter_test.total_mile_truly_use;//��̼Ʋ���		6
			send_d_32[p++]=milemeter_test.total_mile_by_angle;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=SPEED_P_v2.Proportion;//�����ѹ      8

			send_d_16[p++]=SPEED_P_v2.I_Output;//Ŀ��Ƕ�       	9
			send_d_16[p++]=SPEED_P_v2.Integral;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
	#if 0//��̼Ʋ���
		
	p=0;
			send_d_32[p++]=milemeter_test.total_mile_by_angle;//Ŀ��λ��		1
			send_d_32[p++]=milemeter_test.total_mile_by_angle_1000;//��ǰλ��		2

			send_d_32[p++]=milemeter_test.total_mile_by_angle_4000;//Ŀ����̬�Ƕ�		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= milemeter_test.total_mile_by_angle_8191;//I_OUT 4		4PID_YES

			send_d_32[p++]=milemeter_test.total_mile_by_turnCount;//P_OUT		5
		
			send_d_32[p++]=milemeter_test.total_mile_truly_use;//��̼Ʋ���		6
			send_d_32[p++]=milemeter_test.total_mile_by_angle;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=SPEED_P_v2.Proportion;//�����ѹ      8

			send_d_16[p++]=SPEED_P_v2.I_Output;//Ŀ��Ƕ�       	9
			send_d_16[p++]=SPEED_P_v2.Integral;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
				#if 0//Ѱ�һ�е���
		
	p=0;
			send_d_32[p++]=DJIC_IMU.total_pitch*100000;//Ŀ��λ��		1
			send_d_32[p++]=BALANCE_P.Target*100000;//��ǰλ��		2

			send_d_32[p++]=BALANCE_P.Measure*100000;//Ŀ����̬�Ƕ�		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= DR16.rc.ch1*660;//I_OUT 4		4PID_YES

			send_d_32[p++]=DJIC_IMU.total_pitch*1000;//P_OUT		5
			send_d_32[p++]=0;//I_OUT		6
			send_d_32[p++]=0;//D_OUT  	7
	p=0;
			send_d_16[p++]=SPEED_P_v2.Proportion;//�����ѹ      8

			send_d_16[p++]=SPEED_P_v2.I_Output;//Ŀ��Ƕ�       	9
			send_d_16[p++]=SPEED_P_v2.Integral;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
				#if 0//�ٶȻ����
		
	p=0;
			send_d_32[p++]=BALANCE_P.result+BALANCE_I.result;//Ŀ��λ��		1
			send_d_32[p++]=SPEED_P_v2.result;//��ǰλ��		2

			send_d_32[p++]=send_to_tire_L;//Ŀ����̬�Ƕ�		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= DR16.rc.ch1*660;//I_OUT 4		4PID_YES

			send_d_32[p++]=DJIC_IMU.total_pitch*1000;//P_OUT		5
			send_d_32[p++]=milemeter_test.total_mile_by_turnCount;//��̼Ʋ���		6
			send_d_32[p++]=milemeter_test.total_mile_by_angle;//��̼Ʋ���  	7
	p=0;
			send_d_16[p++]=SPEED_P_v2.Proportion;//�����ѹ      8

			send_d_16[p++]=SPEED_P_v2.I_Output;//Ŀ��Ƕ�       	9
			send_d_16[p++]=SPEED_P_v2.Integral;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
		
				#if 0//Ŀ��λ�ÿ���
	p=0;
			send_d_32[p++]=TARGET_position;//Ŀ��λ��		1
			send_d_32[p++]=M3508s[3].totalAngle-M3508s[2].totalAngle;//��ǰλ��		2

			send_d_32[p++]=TARGET_angle_PITCH;//Ŀ����̬�Ƕ�		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= tire_R_TARGE_speed*-1;//I_OUT 4		4PID_YES

			send_d_32[p++]=M3508s[2].realSpeed*-1;//P_OUT		5
			send_d_32[p++]=TARGET_angle_PITCH*1000;//I_OUT		6
			send_d_32[p++]=DJIC_IMU.total_pitch*1000;//D_OUT  	7
	p=0;
			send_d_16[p++]=R_speed_new;//�����ѹ      8

			send_d_16[p++]=yaw_trage_speed*100000;//Ŀ��Ƕ�       	9
			send_d_16[p++]=cali_sensor[i].cali_cmd*1111;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif		
				#if 0//̽���Ƕ����ٶȵĹ�ϵ
	p=0;
			send_d_32[p++]=tire_L_TARGE_speed;//��ǰ�Ƕ�		1
			send_d_32[p++]=M3508s[3].realSpeed;//����Ŀ��Ƕ�		2

			send_d_32[p++]=send_to_tire_L;//�Ӿ�����		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= tire_R_TARGE_speed*-1;//I_OUT 4		4PID_YES

			send_d_32[p++]=M3508s[2].realSpeed*-1;//P_OUT		5
			send_d_32[p++]=TARGET_angle_PITCH*1000;//I_OUT		6
			send_d_32[p++]=DJIC_IMU.total_pitch*1000;//D_OUT  	7
	p=0;
			send_d_16[p++]=R_speed_new;//�����ѹ      8

			send_d_16[p++]=yaw_trage_speed*100000;//Ŀ��Ƕ�       	9
			send_d_16[p++]=cali_sensor[i].cali_cmd*1111;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
#if 0//��������������  YAW PITCH
	p=0;
			send_d_32[p++]=tire_L_TARGE_speed;//��ǰ�Ƕ�		1
			send_d_32[p++]=M3508s[3].realSpeed;//����Ŀ��Ƕ�		2

			send_d_32[p++]=send_to_tire_L;//�Ӿ�����		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= tire_R_TARGE_speed*-1;//I_OUT 4		4PID_YES

			send_d_32[p++]=M3508s[2].realSpeed*-1;//P_OUT		5
			send_d_32[p++]=send_to_tire_R*-1;//I_OUT		6
			send_d_32[p++]=L_speed_new;//D_OUT  	7
	p=0;
			send_d_16[p++]=R_speed_new;//�����ѹ      8

			send_d_16[p++]=yaw_trage_speed*100000;//Ŀ��Ƕ�       	9
			send_d_16[p++]=cali_sensor[i].cali_cmd*1111;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
			#if 0//����  ����  YAW PITCH
	p=0;
			send_d_32[p++]=tire_L_TARGE_speed;//��ǰ�Ƕ�		1
			send_d_32[p++]=M3508s[3].realSpeed;//����Ŀ��Ƕ�		2

			send_d_32[p++]=-tire_R_TARGE_speed;//�Ӿ�����		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= -M3508s[2].realSpeed;//I_OUT 4		4PID_YES

			send_d_32[p++]=tire_R_TARGE_speed_FAKE;//P_OUT		5
			send_d_32[p++]=DJIC_IMU.total_pitch*-10;//I_OUT		6
			send_d_32[p++]=DJIC_IMU.total_pitch*-100;//D_OUT  	7
	p=0;
			send_d_16[p++]=this_period_has_shoot_number;//�����ѹ      8

			send_d_16[p++]=yaw_trage_speed*100000;//Ŀ��Ƕ�       	9
			send_d_16[p++]=cali_sensor[i].cali_cmd*1111;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif
			#if 0//��������������  YAW PITCH
	p=0;
			send_d_32[p++]=INS_angle[0]*10000;//��ǰ�Ƕ�		1
			send_d_32[p++]=INS_angle[1]*10000;//����Ŀ��Ƕ�		2

			send_d_32[p++]=-INS_angle[2]*10000;//�Ӿ�����		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= INS_gyro[0]*10000;//I_OUT 4		4PID_YES

			send_d_32[p++]=INS_gyro[1]*10000;//P_OUT		5
			send_d_32[p++]=INS_gyro[2]*10000;//I_OUT		6
			send_d_32[p++]=DJIC_IMU.total_pitch*-100;//D_OUT  	7
	p=0;
			send_d_16[p++]=this_period_has_shoot_number;//�����ѹ      8

			send_d_16[p++]=yaw_trage_speed*100000;//Ŀ��Ƕ�       	9
			send_d_16[p++]=cali_sensor[i].cali_cmd*1111;//1��У׼ 0����		10
														//������С�������λ558 320 660   bjTlta
#endif


			#if 0//��������������  YAW PITCH
	p=0;
			send_d_32[p++]=INS_accel[1]*1000000;//��ǰ�Ƕ�		1
			send_d_32[p++]=accel_fliter_3[0]*1000000;//����Ŀ��Ƕ�		2

			send_d_32[p++]=accel_fliter_3[1]*1000000;//�Ӿ�����		3 
//				send_d_32[p++]=PID_YES*1000;//P_OUT		3 

			//DJIC_IMU.Gyro_y*1000000
//DJIC_IMU.pitch
			send_d_32[p++]= accel_fliter_3[2]*1000000;//I_OUT 4		4PID_YES

			send_d_32[p++]=DJIC_IMU.add_speed_C*1000000;//P_OUT		5
			send_d_32[p++]=DJIC_IMU.add_speed_Q*1000000;//I_OUT		6
			send_d_32[p++]=DJIC_IMU.total_pitch*-100;//D_OUT  	7
	p=0;
			send_d_16[p++]=this_period_has_shoot_number;//�����ѹ      8

			send_d_16[p++]=yaw_trage_speed*100000;//Ŀ��Ƕ�       	9
			send_d_16[p++]=cali_sensor[i].cali_cmd*1111;//1��У׼ 0����		10
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





