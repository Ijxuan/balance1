/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "calibrate_task.h"

#include "INS_task.h"
#include "led_flow_task.h"
#include "usbd_cdc_if.h"

#include "Debug_DataScope.h"
#include "DJI_IMU.h"
#include "bsp_buzzer.h"
#include "M3508.h"
#include "M2006.h"
#include "my_IncrementPID_bate.h"
#include "DR16_RECIVE.h"
#include "my_positionPID_bate.h"
#include "GM6020.h"
#include "MY_CLOUD_CONTROL.h"
#include "MY_SHOOT_CONTROL.h"
#include "BEEP_MY.h"
#include "bsp_buzzer.h"
#include "Vision.h"
#include "string.h"
#include "FPS_Calculate.h"
#include "bsp_adc.h"
#include "oled.h"
#include "spinning_top_examine.h"
#include "Vision_Control.h"
#include "MY_CLOUD_CONTROL.h"
//#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId calibrate_tast_handle;
osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// CAN���о��
osMessageQId CAN1_Queue;

osMessageQId CAN2_Queue;

//����������
osThreadId RobotCtrl_Handle;
//������ں���
void Robot_Control(void const *argument);

/* USER CODE END Variables */
osThreadId testHandle;
osThreadId Debug_TaskHandle;
osThreadId IMU_Send_TaskHandle;
osThreadId Can2_Rei_TaskHandle;
osThreadId CAN1Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void Debug(void const * argument);
void IMU_Send(void const * argument);
void Can2_Reivece(void const * argument);
void CAN1_R(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
	*ppxTimerTaskStackBuffer = &xTimerStack[0];
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
	/* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	I_PID_Parameter_Init(&Driver_I_PID, 4, 0.2, 5,
						 9000,		  //���ַ���
						 9000, -9000, //������
						 0.5,
						 9000, -9000,
						 10000, -10000); //���̵��
	P_PID_Parameter_Init(&Driver_ANGLE_pid, 0.3, 0, 0,
						 3000,
						 // float max_error, float min_error,
						 //                           float alpha,
						 2000, -2000,
						 7000, -7000);

	I_PID_Parameter_Init(&SHOOT_L_I_PID, 28, 0.35, 15,
						 8700, 7000, -7000,
						 0.5,
						 14000, -14000,
						 16000, -16000); //Ħ���ֵ�� 37 0.35 13

	//
	I_PID_Parameter_Init(&SHOOT_R_I_PID, 28, 0.35, 15,
						 20000, 20000, -20000,
						 0.5,
						 14000, -14000,
						 16000, -16000); //Ħ���ֵ��28 0.35 15 
						 //23 0.5 19
						 //22 0.35 19

#if PID_MOTOR //�Ƿ��������PID
	P_PID_Parameter_Init(&Yaw_Speed_pid, 550, 10, 0,
						 120, //���������ֵ�ͻ��ַ���
						 //	float max_error, float min_error,
						 //                          float alpha,
						 2000, -2000, //�����޷���Ҳ���ǻ��ֵ������Χ
						 29000, -29000);
	P_PID_Parameter_Init(&Yaw_Angle_pid, 0.03, 0, 0,
						 0,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 0, 0,
						 220, -220); //

#endif

#if PID_YAW_IMU
	P_PID_Parameter_Init(&Yaw_IMU_Speed_pid, -800, -4, 800,//
						 60, //���������ֵ�ͻ��ַ���
						 //	float max_error, float min_error,
						 //                          float alpha,
						 0, -0, //�����޷���Ҳ���ǻ��ֵ������Χ
						 29000, -29000);
						 
	P_PID_Parameter_Init(&Yaw_IMU_Angle_pid, 8, 0, 0,//10 0 16//Խ��Խ����10
						 100,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 10, -10,
						 1000, -1000); // Yaw_IMU_Angle_pid
#endif

#if VISION_PID_YAW_IMU


	P_PID_Parameter_Init(&VISION_Yaw_IMU_Speed_pid, -900, -4.5, 800,//
						 60, //���������ֵ�ͻ��ַ���
						 //	float max_error, float min_error,
						 //                          float alpha,
						 5000, -5000, //�����޷���Ҳ���ǻ��ֵ������Χ
						 29000, -29000);
						 
	P_PID_Parameter_Init(&VISION_Yaw_IMU_Angle_pid, 10, 0.085, 0,//10 0 16//Խ��Խ����10
						 2.8,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 600, -600,
						 1000, -1000); // Yaw_IMU_Angle_pid

#endif

#if use_new_gimbal==0
#if PID_PITCH_MOTOR
	P_PID_Parameter_Init(&PITCH_Angle_pid, 0.45, 0, 0,
						 0, //���������ֵ�ͻ��ַ���
						 //	float max_error, float min_error,
						 //                          float alpha,
						 220, -220, //�����޷���Ҳ���ǻ��ֵ������Χ
						 220, -220);
	P_PID_Parameter_Init(&PITCH_Speed_pid, 300, 1, 0,
						 120,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 10000, -10000,	 //�����޷���Ҳ���ǻ��ֵ������Χ
						 29000, -29000); // Yaw_IMU_Angle_pid
#endif

#endif
#if use_new_gimbal==1
#if PID_PITCH_MOTOR
	P_PID_Parameter_Init(&PITCH_Angle_pid, 0.7, 0.05, 0,
						 0, //���������ֵ�ͻ��ַ���
						 //	float max_error, float min_error,
						 //                          float alpha,
						 220, -220, //�����޷���Ҳ���ǻ��ֵ������Χ
						 220, -220);
	P_PID_Parameter_Init(&PITCH_Speed_pid, 300, 1, 0,
						 120,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 10000, -10000,	 //�����޷���Ҳ���ǻ��ֵ������Χ
						 29000, -29000); // Yaw_IMU_Angle_pid
#endif
#endif	
		



#if 0//û�����
	P_PID_Parameter_Init(&PITCH_IMU_Speed_pid, 200,0.4,0,//100, 1.5, 0,
						 15, //���������ֵ�ͻ��ַ���  550 1.9 0   -20000
						 //	float max_error, float min_error,
						 //                          float alpha,
						 3000, -3000, //�����޷���Ҳ���ǻ��ֵ������Χ    80    0.7        5000   -5000     28000   -28000
						 29000, -29000);
	P_PID_Parameter_Init(&PITCH_IMU_Angle_pid,12 //15  //0.7  12
	, 0.1, 0,
						 2.5,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 300, -300,
						 500, -500); // Yaw_IMU_Angle_pid   15    500 -500

#endif
#if PID_PITCH_IMU
//	P_PID_Parameter_Init(&PITCH_IMU_Speed_pid, 150,0.3,0,//100, 1.5, 0,
//						 25, //���������ֵ�ͻ��ַ���  550 1.9 0   -20000
//						 //	float max_error, float min_error,
//						 //                          float alpha,
//						 4000, -4000, //�����޷���Ҳ���ǻ��ֵ������Χ    80    0.7        5000   -5000     28000   -28000
//						 29000, -29000);

	P_PID_Parameter_Init(&PITCH_IMU_Speed_pid, 190,0.9,-200,//100, 1.5, 0,
						 100, //���������ֵ�ͻ��ַ���  550 1.9 0   -20000
						 //	float max_error, float min_error,
						 //                          float alpha,
						 4000, -4000, //�����޷���Ҳ���ǻ��ֵ������Χ    80    0.7        5000   -5000     28000   -28000
						 29000, -29000);
	P_PID_Parameter_Init(&PITCH_IMU_Angle_pid,12 //15  //0.7  12
	, 0.08, 0,
						 2.5,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 300, -300,
						 500, -500); // Yaw_IMU_Angle_pid   15    500 -500

#endif
Vision_Control_Init();//������������ʼ��

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */

		CAN1_Config();
//	CAN1_Filter0_Init();
//	
//	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

//	HAL_CAN_Start(&hcan1);


		CAN2_Config();
			//�Ӿ�
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);

	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

	USART_RX_DMA_ENABLE(&huart6, Vision_DataBuff, Vision_BuffSize);
					ext_robot_hurt.data.hurt_type=10;//�����ʼֵΪ0��ʶ��

//DJIC_IMU.pitch_turnCounts=-1;
	//CAN2_Filter0 ��ʼ�� ʹ��
//	  HAL_Delay(1000);

//	CAN2_Filter0_Init();
//	

//	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

//			HAL_CAN_Start(&hcan2);


  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	CAN2_Queue = xQueueCreate(64, sizeof(CAN_Rx_TypeDef));
	CAN1_Queue = xQueueCreate(32, sizeof(CAN_Rx_TypeDef));
  MX_USB_DEVICE_Init();

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of test */
  osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* definition and creation of Debug_Task */
  osThreadDef(Debug_Task, Debug, osPriorityNormal, 0, 128);
  Debug_TaskHandle = osThreadCreate(osThread(Debug_Task), NULL);

  /* definition and creation of IMU_Send_Task */
  osThreadDef(IMU_Send_Task, IMU_Send, osPriorityHigh, 0, 128);
  IMU_Send_TaskHandle = osThreadCreate(osThread(IMU_Send_Task), NULL);

  /* definition and creation of Can2_Rei_Task */
  osThreadDef(Can2_Rei_Task, Can2_Reivece, osPriorityAboveNormal, 0, 128);
  Can2_Rei_TaskHandle = osThreadCreate(osThread(Can2_Rei_Task), NULL);

  /* definition and creation of CAN1 */
  osThreadDef(CAN1, CAN1_R, osPriorityAboveNormal, 0, 128);
  CAN1Handle = osThreadCreate(osThread(CAN1), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	osThreadDef(cali, calibrate_task, osPriorityAboveNormal, 0, 512);
	calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);

	osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
	imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

	osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 256);
	led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);

//	  osDelay(3000);
//	  osDelay(3000);
//	  osDelay(3000);

	osThreadDef(Task_Robot_Control, Robot_Control, RobotCtrl_Priority, 0, RobotCtrl_Size);
	RobotCtrl_Handle = osThreadCreate(osThread(Task_Robot_Control), NULL);
	PITCH_trage_angle = DJIC_IMU.total_pitch;

	//

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
 * @brief  Function implementing the test thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN test_task */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
  /* USER CODE END test_task */
}

/* USER CODE BEGIN Header_Debug */
/**
 * @brief Function implementing the Debug_Task thread.
 * @param argument: Not used
 * @retval None
 */
//fp32 voltage;
	uint8_t text_send[5];
	char text_e[5]="A432B";

/* USER CODE END Header_Debug */
void Debug(void const * argument)
{
  /* USER CODE BEGIN Debug */
	int debug_times=0;
	char error_e[5]="error";

	uint16_t range_i=0;
	uint16_t give_i=0;

/* 
	char RunTimeInfo[400];		//������������ʱ����Ϣ

char RunTimeInfo[400];		//������������ʱ����Ϣ
������\t\t\t����ʱ��\t������ռ�ٷֱ�
Debug_Task     	427		<1%     ��������λ��
led            	5866		<1%      led
IDLE           	398036		58%  ���� 
IMU_Send_Task  	117005		17%  �����Ӿ�
cali           	63268		9%    У׼ 
test           	455		<1%       ʲô��û��
Task_Robot_Cont	31306		4%    �ܿ���
Can2_Rei_Task  	463		<1%       CAN2����
imuTask        	13982		2%    ����������
CAN1           	49718		7%    CAN1����
Tmr Svc        	0		<1%
		
������\t\t\t����ʱ��\t������ռ�ٷֱ�		
Debug_Task     	4706		<1%
IDLE           	720445		27%
IMU_Send_Task  	1436986		54%
cali           	127828		4%
led            	4709		<1%
test           	457		<1%
Task_Robot_Cont	285106		10%
Tmr Svc        	0		<1%
imuTask        	50511		1%
Can2_Rei_Task  	1		<1%
CAN1           	0		<1%

Debug_Task     	5814		<1%
led            	5804		<1%
IDLE           	885108		27%
IMU_Send_Task  	1764361		54%
cali           	157311		4%
test           	554		<1%
Task_Robot_Cont	350312		10%
Tmr Svc        	0		<1%
imuTask        	62044		1%
Can2_Rei_Task  	1		<1%
CAN1           	0		<1%

		*/
//		buzzer_control.work = TRUE; 
	/* Infinite loop */
	for (;;)
	{
		debug_times++;	
Get_FPS(&FPS_ALL.DEBUG.WorldTimes,&FPS_ALL.DEBUG.FPS);
		
				if(debug_times%10==0)//��λ������Ƶ��
						{
							NM_swj();

							//10msһ��
//			printf("��");
						}
		if (DR16.rc.s_right != 2&&DR16.rc.s_right != 0) //�Ƿ���λ��
		{	

			//					USART1->DR = '2';
			//					TRY[0]='0';
			//										TRY[1]='1';

			//	HAL_UART_Transmit_DMA(&huart1,&TRY[0],2);
						if (cali_sensor[0].cali_done == CALIED_FLAG && cali_sensor[0].cali_cmd == 0)
					{

					}
//		
					
		}	

		
		if(debug_times%100==0)//��λ������Ƶ��
		{
		range_i++;
		if(range_i==1)
		{
		text_e[0]='a';
		text_e[1]='9';
		text_e[2]='8';
		text_e[3]='7';
		text_e[4]='b';

		}
		if(range_i==2)
		{
		text_e[0]='c';
		text_e[1]='1';
		text_e[2]='2';
		text_e[3]='3';
		text_e[4]='d';
		range_i=0;
		}
		for(give_i=0;give_i<5;give_i++)
		{
		text_send[give_i]=(char )(text_e[give_i]);
		}			
			
//			CDC_Transmit_FS(&text_send[0],5);
		}	
						
		if(DR16.rc.s_left == 1)
		{
			if(in_END==1&&in_END_last==0)//��һʱ�̲��ڹ��ĩ��,��һʱ���ڹ��ĩ��
			{
		stay_in_track_end_times++;//1ms����һ��
			}
			if(stay_in_track_end_times>0)
			{
			stay_in_track_end_times++;
			}
			if(stay_in_track_end_times>1600)
			{
			stay_in_track_end_times=0;
			}
			if(in_END==0)
			{
			stay_in_track_end_times=0;//��һʱ�̲��ڹ��ĩ����,�ǿ϶��Ͳ���Ҫ��ʱ��,��ʱ����
			}
		}
		in_END_last=in_END;
		if(debug_times%100==0)//100ms����һ��
			
		{
		        //get battery voltage
        //��ȡ��Դ��ѹ
			
//        my_voltage = get_battery_voltage();
			if(my_voltage<21&&my_voltage>20)
			{
			  		if(debug_times%5000==0)//5s����һ��
					{
												Buzzer.mode = One_times;

					}
			}
			
		}
  		if(debug_times%1000==0)//1s����һ��
		{
//		    oled_clear(Pen_Clear);
//oled_printf(1,1,error_e);
//	    oled_refresh_gram();
	
		}
		  

		  
   #ifdef FREERTOS_TASK_TIME
		  		if(debug_times%1000==0)//1s����һ��
				{
				memset(RunTimeInfo,0,400);				//��Ϣ����������

			vTaskGetRunTimeStats(RunTimeInfo);		//��ȡ��������ʱ����Ϣ
		  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&RunTimeInfo, 400);	
				}
#endif	  
		
		
//BEEP_TEXT();		
		
		  laoliu_gjiwo_times_ago++;//����������
		hurt_times_ago++;
				if(ext_robot_hurt.data.hurt_type==0)
		{
			if(ext_robot_hurt.data.armor_id==0||ext_robot_hurt.data.armor_id==1)
			{
				hurt_times_ago=0;//��������
			
			if(ext_robot_hurt.data.armor_id==1)//����װ�װ��ܻ�
			{
			laoliu_gjiwo_times_ago=0;//����������
			
			}
				ext_robot_hurt.data.hurt_type=10;
			}

		}
		osDelay(1);
	}
  /* USER CODE END Debug */
}

/* USER CODE BEGIN Header_IMU_Send */
/**
 * @brief Function implementing the IMU_Send_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IMU_Send */
void IMU_Send(void const * argument)
{
  /* USER CODE BEGIN IMU_Send */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //ÿʮ����ǿ�ƽ����ܿ���
	/* Infinite loop */
	for (;;)
	{
#if send_way == 0
		//ŷ����
//		if (cali_sensor[0].cali_done == CALIED_FLAG && cali_sensor[0].cali_cmd == 0)
//		{
//			Euler_Send.yaw = INS_angle[0];
//			Euler_Send.pitch = INS_angle[1];
//			Euler_Send_Fun(Euler_Send);
//			//���ٶ�
//			Gyro_Send.Gyro_z = INS_gyro[2];
//			Gyro_Send.Gyro_y = INS_gyro[1];
//			Gyro_Send_Fun(Gyro_Send);
//		}
//			DR16_T.DR16_CH0=DR16.rc.ch0=-100;
//			DR16_T.DR16_CH1=DR16.rc.ch1=-200;
//			DR16_T.DR16_CH2=DR16.rc.ch2=-300;
//			DR16_T.DR16_CH3=DR16.rc.ch3=-400;
//		DR16_Send_Fun(DR16_T);

#endif
Update_Vision_SendData();
		VISION_Disconnect_test++;
		if(VISION_Disconnect_test==1000)//һ����һ��
		{
			if(VisionData.Offline_Detec>10)//һ�����10�β����ְ�
				VisionData.Offline_Detec=0;
			else
			{
				VisionData.Offline_Detec=0;
				VisionData.DataUpdate_Flag=0;//��־λ����
				VisionData.RawData.Beat=0;//��������
				VisionData.RawData.Armour=0;//ʶ������
				VisionData.RawData.Pitch_Angle=0;
				VisionData.RawData.Yaw_Angle=0;
			}
			VISION_Disconnect_test=0;
		}
		
		
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
	}
  /* USER CODE END IMU_Send */
}

/* USER CODE BEGIN Header_Can2_Reivece */
/**
 * @brief Function implementing the Can2_Rei_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Can2_Reivece */
void Can2_Reivece(void const * argument)
{
  /* USER CODE BEGIN Can2_Reivece */
	/* Infinite loop */
	/* USER CODE BEGIN Can2_Reivece */
	// CAN2���ն��з��͵����ݵĽṹ�����
	CAN_Rx_TypeDef CAN2_Rx_Structure;
	//���ӵ�״̬����
	BaseType_t ExitQueue_Status;
int i=0;
	/* Infinite loop */
	for (;;)
	{
		//���ȶ�������Ϣ
		ExitQueue_Status = xQueueReceive(CAN2_Queue, &CAN2_Rx_Structure, portMAX_DELAY);
		//���ӳɹ�
		if (ExitQueue_Status == pdTRUE)
		{
			CAN2_rc_times++;
			can2_DR16_TIMES++;
			//������У׼ָ��
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == IMU_CAL_REIID)
			{
				//���
				IMU_Cal_Status_Reivece(CAN2_Rx_Structure);
			}
			
						if (CAN2_Rx_Structure.CAN_RxMessage.StdId == ENCODER_ID)
			{//������λ����Ϣ���
				
								for(i=0;i<8;i++)
				{
				Chassis_Encoder_new.data.dataBuff[i]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				Chassis_Encoder_new.infoUpdateFlag++;
			}
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == PLACE_SEND_ID)
			{//����λ����Ϣ���
				
				place_complete_update_TIMES++;
								for(i=0;i<8;i++)
				{
				CHASSIS_place[i]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				
				if(CHASSIS_place[0]==1||CHASSIS_place[7]==1)
				{
					in_END=1;
				}
				if(CHASSIS_place[0]==0&&CHASSIS_place[7]==0)
				{
					in_END=0;
				}
				
				if(CHASSIS_place[0]==1)
				{
					in_END_L=1;

				}
				else
				{
					in_END_L=0;					
				}
				
				if(CHASSIS_place[7]==1)
				{
					in_END_R=1;

				}
				else
				{
					in_END_R=0;
				}
				


				if(CHASSIS_place[3]==1||CHASSIS_place[4]==1)
				{
					in_MID=1;
				}
				if(CHASSIS_place[3]==0||CHASSIS_place[4]==0)
				{
					in_MID=0;
				}
					Get_FPS(&FPS_ALL.CHASSIS_PLACE.WorldTimes,   &FPS_ALL.CHASSIS_PLACE.FPS);

			}
			
						if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_HP_ID_ONE)
			{
				//HP���ݽ��1
				for(i=0;i<8;i++)
				{
				ext_game_robot_HP.data.dataBuff[i]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				STATUS_PART_ONE_TIMES++;

			}
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_HP_ID_TWO)
			{
				//HP���ݽ��2
				for(i=0;i<8;i++)
				{
				ext_game_robot_HP.data.dataBuff[i+8]=CAN2_Rx_Structure.CAN_RxMessageData[i];

				}
				STATUS_PART_TWO_TIMES++;

			}
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_HP_ID_THREE)
			{
				//HP���ݽ��3
				for(i=0;i<8;i++)
				{
				ext_game_robot_HP.data.dataBuff[i+16]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				STATUS_PART_THREE_TIMES++;
			}	
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_HP_ID_FOUR)
			{
				//HP���ݽ��4
				for(i=0;i<8;i++)
				{
				ext_game_robot_HP.data.dataBuff[i+24]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
HP_complete_update_TIMES++;
				}	
			
				if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_HGAME_STATE_ID_ONE)
			{
				//�������ݽ��1
				for(i=0;i<8;i++)
				{
				ext_game_status.data.dataBuff[i]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
			}
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_HGAME_STATE_ID_TWO)
			{
				//�������ݽ��2
				for(i=0;i<8;i++)
				{
				ext_game_status.data.dataBuff[i+8]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				GAME_STATE_update_TIMES++;
			}		
			
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_HEAT_ID_ONE)
			{
				//�������ݽ��1
				for(i=0;i<8;i++)
				{
				ext_power_heat_data.data.dataBuff[i]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
			}
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_HEAT_ID_TWO)
			{
				//�������ݽ��2
				for(i=0;i<8;i++)
				{
				ext_power_heat_data.data.dataBuff[i+8]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				HEAT_complete_update_TIMES++;
			}
			
			
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_RC_SHOOT_ID)
			{
				//���1
				for(i=0;i<8;i++)
				{
				ext_shoot_data.data.dataBuff[i]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				JS_RC_TIMES++;
//run_JS_jiema=1;
			}
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_RC_HURT_ID)
			{
				//���
				ext_robot_hurt.data.dataBuff[0]=CAN2_Rx_Structure.CAN_RxMessageData[0];
				ext_robot_hurt.InfoUpdataFlag++;
			}		
			
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_STATUS_ID_ONE)
			{
				//״̬���ݽ��1
				for(i=0;i<8;i++)
				{
				ext_game_robot_state.data.dataBuff[i]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				STATUS_PART_ONE_TIMES++;

			}
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_STATUS_ID_TWO)
			{
				//״̬���ݽ��2
				for(i=0;i<8;i++)
				{
				ext_game_robot_state.data.dataBuff[i+8]=CAN2_Rx_Structure.CAN_RxMessageData[i];

				}
				STATUS_PART_TWO_TIMES++;

			}
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_STATUS_ID_THREE)
			{
				//״̬���ݽ��3
				for(i=0;i<8;i++)
				{
				ext_game_robot_state.data.dataBuff[i+16]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				STATUS_PART_THREE_TIMES++;
			}	
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == JS_SEND_STATUS_ID_FOUR)
			{
				//״̬���ݽ��4
				for(i=0;i<8;i++)
				{
				ext_game_robot_state.data.dataBuff[i+24]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				STATUS_PART_FOUR_TIMES++;
STATUS_complete_update_TIMES++;
				}			
			
			
			
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == DR16_R_PART_ONE)
			{
				//���1
				for(i=0;i<8;i++)
				{
				DR16Buffer[i]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				DDR16_PART_ONE_TIMES++;

			}
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == DR16_R_PART_TWO)
			{
				//���2
				for(i=0;i<8;i++)
				{
				DR16Buffer[i+8]=CAN2_Rx_Structure.CAN_RxMessageData[i];

				}
				DDR16_PART_TWO_TIMES++;

			}
			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == DR16_R_PART_THREE)
			{
				//���3
				for(i=0;i<8;i++)
				{
				DR16Buffer[i+16]=CAN2_Rx_Structure.CAN_RxMessageData[i];
				}
				DDR16_PART_THREE_TIMES++;
				run_DR16_jiema=1;
			}
			
		if(run_DR16_jiema==1)
		{
							DR16.DR16_Process(DR16Buffer);

			run_DR16_jiema=0;
		}
			
		}
	}
	//    osDelay(1);
  /* USER CODE END Can2_Reivece */
}

/* USER CODE BEGIN Header_CAN1_R */
/**
 * @brief Function implementing the CAN1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CAN1_R */
void CAN1_R(void const * argument)
{
  /* USER CODE BEGIN CAN1_R */
	// CAN1���ն��з��͵����ݵĽṹ�����
	CAN_Rx_TypeDef CAN1_Rx_Structure;

	//���ӵ�״̬����
	BaseType_t ExitQueue_Status;
	/* Infinite loop */
	for (;;)
	{
		//���ȶ�������Ϣ
		ExitQueue_Status = xQueueReceive(CAN1_Queue, &CAN1_Rx_Structure, portMAX_DELAY);
		//���ӳɹ�

		if (ExitQueue_Status == pdTRUE)
		{
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (M3508_READID_START + 3))
			{
				//��Ħ���ֵ�    ID1
				M3508s1_getInfo(CAN1_Rx_Structure); //
			}
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (M3508_READID_START + 2))
			{
				//��Ħ���ֵ�	ID2
				M3508s1_getInfo(CAN1_Rx_Structure); //
			}
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (M3508_READID_START + 1))
			{
				//���̵��		ID3
				M2006_getInfo(CAN1_Rx_Structure); //
			}

			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == GM6020_READID_START)
			{
				//��̨
				GM6020_Yaw_getInfo(CAN1_Rx_Structure);
			}

			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (GM6020_READID_START + 1))
			{
				//��̨��yaw��
				GM6020_Pitch_getInfo(CAN1_Rx_Structure); // 6400-7160
			}

			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == GM6020_READID_START + 2)
			{
				//��̨
				GM6020_Yaw_getInfo(CAN1_Rx_Structure);
			}

			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == GM6020_READID_END)
			{
				//��̨��pitch��3743-4557
				GM6020_Pitch_getInfo(CAN1_Rx_Structure); // 6400-7160
			}
		}
	}
  /* USER CODE END CAN1_R */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
uint8_t TRY[20]; //������λ����������

int Driver_add = 32768;

void Robot_Control(void const *argument)
{
	/* USER CODE BEGIN RobotControl */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1); //ÿ2����ǿ�ƽ����ܿ���
//    vTaskDelay(1000);
//    vTaskDelay(1000);
//    vTaskDelay(1000);
//    vTaskDelay(1000);
//    vTaskDelay(1000);
//    vTaskDelay(1000);
//    vTaskDelay(1000);
//    vTaskDelay(1000);
//    vTaskDelay(1000);
//    vTaskDelay(1000);
    vTaskDelay(1000);
    vTaskDelay(1000);
	yaw_trage_angle=DJIC_IMU.total_yaw;
	PITCH_trage_angle = DJIC_IMU.total_pitch;
M2006_targe_angle=M3508s[1].totalAngle;//�������Ŀ��Ƕ��ۼ�

    vTaskDelay(1000);
    vTaskDelay(1000);

	GM6020s[3].turnCount=0;
yaw_trage_angle=DJIC_IMU.total_yaw;
	PITCH_trage_angle = DJIC_IMU.total_pitch;
YAW_MOTION_STATE=1;//����С���ݼ��
PITCH_trage_angle_motor=GM6020s[3].totalAngle;
//key_message.game_state_progress=4;
//ext_game_status.data.game_progress=4;
//key_message.our_outpost_is_live=1;
	/* Infinite loop */
	for (;;)
	{
			#if SHOOT_HIGH_HEAT_TEXT
		vision_shoot_times=100;
VisionData.RawData.Armour=1;
				if(controul_times%200==0)
		{
	VisionData.RawData.Beat=1;		
		}
		else
		{
				VisionData.RawData.Beat=0;		
	}
//VisionData.RawData.Beat=1;
	Vision_RawData_Pitch_Angle=0;
	Vision_RawData_Yaw_Angle=0;
#endif

						Vision_Control_Cloud();

		
								if (DR16.rc.s_left == 2&&DR16.rc.ch3<-600) //ʧ�ܱ���
				{
					disable_for_test_CHASSIS=1;
				}
				if (DR16.rc.s_left == 2&&DR16.rc.ch3>600) //ʧ�ܱ���
				{
					disable_for_test_CHASSIS=0;
				}	
		if(controul_times%1000==0)
		{	
			yaw_trage_angle_add_1s=yaw_trage_angle-yaw_trage_angle_1s_ago;
			yaw_trage_angle_1s_ago=yaw_trage_angle;	
		}
		if(controul_times%10==0)
		{
			 if(DR16.rc.s_left==3&&DR16.rc.s_right==3)
			 {
//		yaw_trage_angle=simulation_target_yaw;
			 }
		}
				if (DR16.rc.s_left == 2&&DR16.rc.ch1<-600) //ʧ�ܱ���
				{
					disable_for_test=1;
				}
				if (DR16.rc.s_left == 2&&DR16.rc.ch1>600) //ʧ�ܱ���
				{
					disable_for_test=0;
				}					

		


				if(VisionData.RawData.Armour==0)//
				{
		Armour_lose_time++;
				}
			else if(VisionData.RawData.Armour==1)//
				{
		Armour_lose_time=0;
				}
				
				
				
				
				controul_times++;
				S_T_examine();
		cloud_control();

				
#if use_new_gimbal==0
		if (GM6020s[3].totalAngle <= 3860 && send_to_pitch < 0)//3860
			send_to_pitch = 0;
		if (GM6020s[3].totalAngle >= 5130 && send_to_pitch > 0)
			send_to_pitch = 0;			
#endif
				#if use_new_gimbal==1

		if (GM6020s[3].totalAngle <= 5150 && send_to_pitch < 0)//3860
			send_to_pitch = 0;
		if (GM6020s[3].totalAngle >= 6550 && send_to_pitch > 0)
			send_to_pitch = 0;
#endif
			
		//��̨��pitch����ֱ�����ϵ�
		////8017-7044
//			send_to_yaw = 0;


		if (DR16.rc.s_left == 2 || DR16.rc.s_left == 0) //ʧ�ܱ���
		{	
			send_to_pitch = 0;
			send_to_yaw = 0;

			PITCH_trage_angle=DJIC_IMU.total_pitch;
			yaw_trage_angle=DJIC_IMU.total_yaw;
			PITCH_trage_angle_motor=GM6020s[3].totalAngle;
			//��������ʹ��˲��ᶶһ�£�Ӧ�û�Ҫ���I���ۼӣ��Ժ���ʱ����д
		}
		if(disable_for_test==1)
		{

			send_to_pitch = 0;			
				send_to_yaw = 0;
			
		}
	
		GM6020_SetVoltage(send_to_yaw,0 , 0, send_to_pitch); //��̨  send_to_pitch
//		GM6020_SetVoltage(0,0 , 0, 0); //��̨  send_to_pitch

		//	if(DR16.rc.s_left==1)//ң��������  ����
		// SHOOT_L_speed=500;
		//	if(DR16.rc.s_left==3)//ң��������  ����
		//					{
		////SHOOT_L_speed=(DR16.rc.ch3*1.0/660.0)*(-1)*8000;//ң�������ٶ�Ŀ��ֵ ��ѡһ
		// if(DR16.rc.ch3<-600)
		//			SHOOT_L_speed=6700;
		// else
		//				SHOOT_L_speed=0;

		//						}
		//				SHOOT_R_speed=SHOOT_L_speed;
		//		if(	SHOOT_L_speed>0)
		//		SHOOT_L_speed=-SHOOT_L_speed;//��Ħ�����ٶ�Ŀ��ֵӦ���Ǹ�ֵ
		//		if(	SHOOT_R_speed<0)
		//		SHOOT_R_speed=-SHOOT_R_speed;//��Ħ�����ٶ�Ŀ��ֵӦ������ֵ
		// send_to_SHOOT_L=I_PID_Regulation(&SHOOT_L_I_PID,SHOOT_R_speed,M3508s[3].realSpeed);

		// send_to_SHOOT_R=I_PID_Regulation(&SHOOT_R_I_PID,SHOOT_L_speed,M3508s[2].realSpeed);
		//

		shoot_control();
//	shoot_control_V2();
		if (DR16.rc.s_left == 2 || DR16.rc.s_left == 0) //ʧ�ܱ���
		{
			//						send_to_chassis=0;
	send_to_SHOOT_L = 0;
			send_to_SHOOT_R = 0;	
	GM6020s[3].turnCount=0;
DJIC_IMU.yaw_turnCounts=0;
			send_to_2006 = 0;
			//						send_to_yaw=0;
			//						M2006_targe_angle=M3508s[1].totalAngle;//Ħ�����������
		}
				if(disable_for_test==1)
		{
								send_to_SHOOT_L = 0;
			send_to_SHOOT_R = 0;	
						send_to_2006 = 0;

		}


		M3508s1_setCurrent(0, send_to_2006, send_to_SHOOT_R, send_to_SHOOT_L);//send_to_SHOOT_L������
//		M3508s1_setCurrent(0, 0,TEST_Current_R ,TEST_Current_L );  //end_to_SHOOT_R����������
        //                                    +                 -
		vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		
	}

	/* USER CODE END RobotControl */
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
