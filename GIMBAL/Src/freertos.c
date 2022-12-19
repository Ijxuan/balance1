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
#include "MY_balance_CONTROL.h"
#include "LPF.h"
#include "CAN2_SEND.h"
#include "MIT.h"

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
// CAN队列句柄
osMessageQId CAN1_Queue;

osMessageQId CAN2_Queue;

//控制任务句柄
osThreadId RobotCtrl_Handle;
//任务入口函数
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
						 9000,		  //积分分离
						 9000, -9000, //最大误差
						 0.5,
						 9000, -9000,
						 10000, -10000); //拨盘电机
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
						 16000, -16000); //摩擦轮电机 37 0.35 13

	//
	I_PID_Parameter_Init(&SHOOT_R_I_PID, 28, 0.35, 15,
						 20000, 20000, -20000,
						 0.5,
						 14000, -14000,
						 16000, -16000); //摩擦轮电机28 0.35 15 
						 //23 0.5 19
						 //22 0.35 19

#if PID_MOTOR //是否开启电机的PID
	P_PID_Parameter_Init(&Yaw_Speed_pid, 550, 10, 0,
						 120, //误差大于这个值就积分分离
						 //	float max_error, float min_error,
						 //                          float alpha,
						 2000, -2000, //积分限幅，也就是积分的输出范围
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
						 60, //误差大于这个值就积分分离
						 //	float max_error, float min_error,
						 //                          float alpha,
						 0, -0, //积分限幅，也就是积分的输出范围
						 29000, -29000);
						 
	P_PID_Parameter_Init(&Yaw_IMU_Angle_pid, 8, 0, 0,//10 0 16//越大越陡峭10
						 100,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 10, -10,
						 1000, -1000); // Yaw_IMU_Angle_pid
#endif

#if VISION_PID_YAW_IMU


	P_PID_Parameter_Init(&VISION_Yaw_IMU_Speed_pid, -900, -4.5, 800,//
						 60, //误差大于这个值就积分分离
						 //	float max_error, float min_error,
						 //                          float alpha,
						 5000, -5000, //积分限幅，也就是积分的输出范围
						 29000, -29000);
						 
	P_PID_Parameter_Init(&VISION_Yaw_IMU_Angle_pid, 10, 0.085, 0,//10 0 16//越大越陡峭10
						 2.8,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 600, -600,
						 1000, -1000); // Yaw_IMU_Angle_pid

#endif

#if use_new_gimbal==0
#if PID_PITCH_MOTOR
	P_PID_Parameter_Init(&PITCH_Angle_pid, 0.45, 0, 0,
						 0, //误差大于这个值就积分分离
						 //	float max_error, float min_error,
						 //                          float alpha,
						 220, -220, //积分限幅，也就是积分的输出范围
						 220, -220);
	P_PID_Parameter_Init(&PITCH_Speed_pid, 300, 1, 0,
						 120,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 10000, -10000,	 //积分限幅，也就是积分的输出范围
						 29000, -29000); // Yaw_IMU_Angle_pid
#endif

#endif
#if use_new_gimbal==1
#if PID_PITCH_MOTOR
	P_PID_Parameter_Init(&PITCH_Angle_pid, 0.7, 0.05, 0,
						 0, //误差大于这个值就积分分离
						 //	float max_error, float min_error,
						 //                          float alpha,
						 220, -220, //积分限幅，也就是积分的输出范围
						 220, -220);
	P_PID_Parameter_Init(&PITCH_Speed_pid, 300, 1, 0,
						 120,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 10000, -10000,	 //积分限幅，也就是积分的输出范围
						 29000, -29000); // Yaw_IMU_Angle_pid
#endif
#endif	
		



#if 0//没电参数
	P_PID_Parameter_Init(&PITCH_IMU_Speed_pid, 200,0.4,0,//100, 1.5, 0,
						 15, //误差大于这个值就积分分离  550 1.9 0   -20000
						 //	float max_error, float min_error,
						 //                          float alpha,
						 3000, -3000, //积分限幅，也就是积分的输出范围    80    0.7        5000   -5000     28000   -28000
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
//						 25, //误差大于这个值就积分分离  550 1.9 0   -20000
//						 //	float max_error, float min_error,
//						 //                          float alpha,
//						 4000, -4000, //积分限幅，也就是积分的输出范围    80    0.7        5000   -5000     28000   -28000
//						 29000, -29000);

	P_PID_Parameter_Init(&PITCH_IMU_Speed_pid, 190,0.9,-200,//100, 1.5, 0,
						 100, //误差大于这个值就积分分离  550 1.9 0   -20000
						 //	float max_error, float min_error,
						 //                          float alpha,
						 4000, -4000, //积分限幅，也就是积分的输出范围    80    0.7        5000   -5000     28000   -28000
						 29000, -29000);
	P_PID_Parameter_Init(&PITCH_IMU_Angle_pid,12 //15  //0.7  12
	, 0.08, 0,
						 2.5,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 300, -300,
						 500, -500); // Yaw_IMU_Angle_pid   15    500 -500

#endif


	P_PID_Parameter_Init(&TIRE_L_SPEED_pid
	,20 //15  //0.7  12
	,2,
	40,
						 300,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 10000, -10000,
						 16000, -16000); // Yaw_IMU_Angle_pid   15    500 -500
	
	P_PID_Parameter_Init(&TIRE_R_SPEED_pid
	,20 //15  //0.7  12
	,2,
	40,
						 300,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 10000, -10000,
						 16000, -16000); // Yaw_IMU_Angle_pid   15    500 -500
						 
#if 0//第一代参数
	P_PID_Parameter_Init(&BALANCE_P,280,0,0,4,//60,0.5,-30,0,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 700, -700,
						 14000, -14000); // 平衡PID
	P_PID_Parameter_Init(&BALANCE_I,40,0,0,0,//60,0.5,-30,0,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 500, -500,
						 14000, -14000); // 平衡PID
	P_PID_Parameter_Init(&SPEED_P,-0.8,-0.15,0,7300,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 2000, -2000,
						 6000, -6000); // 速度PID
						 
	P_PID_Parameter_Init(&change_direction_angle,6,0,0,0,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 0, -0,
						 300, -300); // 转向环角度PID	
	P_PID_Parameter_Init(&change_direction_speed,90,0,0,7300,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 2000, -2000,
						 6000, -6000); // 转向环速度PID						 
#endif
#if 0//第2代参数
	P_PID_Parameter_Init(&BALANCE_P,280,0,0,4,//60,0.5,-30,0,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 700, -700,
						 14000, -14000); // 平衡PID
	P_PID_Parameter_Init(&BALANCE_I,40,0,0,0,//60,0.5,-30,0,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 500, -500,
						 14000, -14000); // 平衡PID
	P_PID_Parameter_Init(&SPEED_P,-0.5,-0.15,0,7300,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 2000, -2000,
						 6000, -6000); // 速度PID
						 
	P_PID_Parameter_Init(&change_direction_angle,6,0,0,0,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 0, -0,
						 300, -300); // 转向环角度PID	
	P_PID_Parameter_Init(&change_direction_speed,90,0,0,7300,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 2000, -2000,
						 6000, -6000); // 转向环速度PID						 
#endif
#if 0//第3代参数 原地抽搐
	P_PID_Parameter_Init(&BALANCE_P,500,0,0,4,//60,0.5,-30,0,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 700, -700,
						 14000, -14000); // 平衡PID
	P_PID_Parameter_Init(&BALANCE_I,40,0,0,0,//60,0.5,-30,0,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 500, -500,
						 14000, -14000); // 平衡PID
	P_PID_Parameter_Init(&SPEED_P,-1.5,-0.3,0,7300,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 3500, -3500,
						 10000, -10000); // 速度PID
		SPEED_P.max_error=30;
		SPEED_P.min_error=-30;


P_PID_V2_Init(&SPEED_P_v2,-2,-0.5,1.5,7300,//-0.5  -0.15软
						3000,-3000, //						  float max_error, float min_error,
						20,-20, //                          float alpha,
						 1500, -1500,
						 10000, -10000); // 速度PIDV2

	P_PID_Parameter_Init(&change_direction_angle,6,0,0,0,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 0, -0,
						 300, -300); // 转向环角度PID	
	P_PID_Parameter_Init(&change_direction_speed,90,0,0,7300,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 2000, -2000,
						 6000, -6000); // 转向环速度PID						 
#endif
#if 0//第4代参数 软
	P_PID_Parameter_Init(&BALANCE_P,500,0,0,4,//60,0.5,-30,0,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 700, -700,
						 14000, -14000); // 平衡PID
	P_PID_Parameter_Init(&BALANCE_I,40,0,0,0,//60,0.5,-30,0,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 500, -500,
						 14000, -14000); // 平衡PID
	P_PID_Parameter_Init(&SPEED_P,-1.5,-0.1,0,7300,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 3500, -3500,
						 10000, -10000); // 速度PID
		SPEED_P.max_error=30;
		SPEED_P.min_error=-30;


P_PID_V2_Init(&SPEED_P_v2,-1.5,-0.1,1,7300,//-0.5  -0.15软
						3000,-3000, //						  float max_error, float min_error,
						10,-10, //                          float alpha,
						 1500, -1500,
						 10000, -10000); // 速度PIDV2
P_PID_V2_Init(&POSITION_v2,-2,0,0,7300,//-0.5  -0.15软
						999,-999, //						  float max_error, float min_error,
						10,-10, //                          float alpha,
						 1500, -1500,
						 6666, -6666); // 速度PIDV2

	P_PID_Parameter_Init(&change_direction_angle,6,0,0,0,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 0, -0,
						 555, -555); // 转向环角度PID	
	P_PID_Parameter_Init(&change_direction_speed,90,0,0,7300,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 2000, -2000,
						 6000, -6000); // 转向环速度PID						 
#endif
#if 1//第5代参数 软
	P_PID_Parameter_Init(&BALANCE_P,500,0,0,4,//60,0.5,-30,0,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 700, -700,
						 14000, -14000); // 平衡PID
	P_PID_Parameter_Init(&BALANCE_I,40,0,0,0,//60,0.5,-30,0,
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 500, -500,
						 14000, -14000); // 平衡PID
	P_PID_Parameter_Init(&SPEED_P,-1.5,0,1.5,7300,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 3500, -3500,
						 10000, -10000); // 速度PID
		SPEED_P.max_error=30;
		SPEED_P.min_error=-30;


P_PID_V2_Init(&SPEED_P_v2,-1.5,0,1.5,7300,//-0.5  -0.15软
						3000,-3000, //						  float max_error, float min_error,
						10,-10, //                          float alpha,
						 1500, -1500,
						 10000, -10000); // 速度PIDV2
P_PID_V2_Init(&POSITION_v2,-2,0,0,7300,//-0.5  -0.15软
						999,-999, //						  float max_error, float min_error,
						10,-10, //                          float alpha,
						 1500, -1500,
						 6666, -6666); // 速度PIDV2

	P_PID_Parameter_Init(&change_direction_angle,6,0,0,0,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 0, -0,
						 700, -7700); // 转向环角度PID	
	P_PID_Parameter_Init(&change_direction_speed,40,0,0,7300,//-0.5  -0.15软
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 2000, -2000,
						 15000, -15000); // 转向环速度PID						 
#endif
SPEED_L.LPF_K=0.85;
SPEED_R.LPF_K=0.85;
milemeter_A.LPF_K=0.6;
ZX.Rate=1;
ZX.Absolute_Max=660;
	P_PID_Parameter_Init(&RC_SPEED_TO_POSITION,0.8,0,0,0,//-0.00001
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 0, -0,
						 160000, -160000); // //位置环（输入目标位置,得到倾斜角度）						 


	P_PID_Parameter_Init(&POSITION,0,0,0,0,//-0.00001
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 0, -0,
						 12, -12); // //位置环（输入目标位置,得到倾斜角度）						 

	P_PID_Parameter_Init(&MIT_TEXT,25,0.3,0,100,//-0.00001
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 10, -10,
						 35, -35); // //MIT电机  角度环得到速度
		P_PID_Parameter_Init(&MIT_SPEED_TEXT,0.8,0.03,-0.6,4,//-0.00001
	 //						  float max_error, float min_error,
	 //                          float alpha,
	 555, -555,
	 80, -80); // //MIT电机 速度环
		P_PID_Parameter_Init(&MIT_POSITION_TEXT,1,0.1,0,100,//-0.00001
	 //						  float max_error, float min_error,
	 //                          float alpha,
	 500, -500,
	 80, -80); // //MIT电机 位置环	 
	 
	MIT_PID_INIT(); 
	 SPEED_MIT_A.LPF_K=0.04;
	 SPEED_MIT_B.LPF_K=0.04;
	 SPEED_MIT_C.LPF_K=0.04;
	 SPEED_MIT_D.LPF_K=0.04;
	 
	 
SPEED_MIT.LPF_K=0.04;
//Vision_Control_Init();//卡尔曼参数初始化   TIRE_L_SPEED_pid   BALANCE_I

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */

		CAN1_Config();
//	CAN1_Filter0_Init();
//	
//	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

//	HAL_CAN_Start(&hcan1);


		CAN2_Config();
			//视觉
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);

	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

	USART_RX_DMA_ENABLE(&huart6, Vision_DataBuff, Vision_BuffSize);
					ext_robot_hurt.data.hurt_type=10;//避免初始值为0误识别

//DJIC_IMU.pitch_turnCounts=-1;
	//CAN2_Filter0 初始化 使能
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
	char RunTimeInfo[400];		//保存任务运行时间信息

char RunTimeInfo[400];		//保存任务运行时间信息
任务名\t\t\t运行时间\t运行所占百分比
Debug_Task     	427		<1%     反馈给上位机
led            	5866		<1%      led
IDLE           	398036		58%  空闲 
IMU_Send_Task  	117005		17%  发给视觉
cali           	63268		9%    校准 
test           	455		<1%       什么都没做
Task_Robot_Cont	31306		4%    总控制
Can2_Rei_Task  	463		<1%       CAN2接收
imuTask        	13982		2%    陀螺仪任务
CAN1           	49718		7%    CAN1接收
Tmr Svc        	0		<1%
		
任务名\t\t\t运行时间\t运行所占百分比		
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
		
				if(debug_times%2==0)//上位机发送频率
						{
							NM_swj();

							//2ms一次
//			printf("好");
						}
		if (DR16.rc.s_right != 2&&DR16.rc.s_right != 0) //是否上位机
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

		
		if(debug_times%100==0)//上位机发送频率
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
			if(in_END==1&&in_END_last==0)//上一时刻不在轨道末端,这一时刻在轨道末端
			{
		stay_in_track_end_times++;//1ms增加一次
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
			stay_in_track_end_times=0;//这一时刻不在轨道末端了,那肯定就不需要计时了,计时清零
			}
		}
		in_END_last=in_END;
		if(debug_times%100==0)//100ms运行一次
			
		{
		        //get battery voltage
        //获取电源电压
			
//        my_voltage = get_battery_voltage();
			if(my_voltage<21&&my_voltage>20)
			{
			  		if(debug_times%5000==0)//5s运行一次
					{
												Buzzer.mode = One_times;

					}
			}
			
		}
  		if(debug_times%1000==0)//1s运行一次
		{
//		    oled_clear(Pen_Clear);
//oled_printf(1,1,error_e);
//	    oled_refresh_gram();
	
		}
		  

		  
   #ifdef FREERTOS_TASK_TIME
		  		if(debug_times%1000==0)//1s运行一次
				{
				memset(RunTimeInfo,0,400);				//信息缓冲区清零

			vTaskGetRunTimeStats(RunTimeInfo);		//获取任务运行时间信息
		  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&RunTimeInfo, 400);	
				}
#endif	  
		
		
//BEEP_TEXT();		
		
		  laoliu_gjiwo_times_ago++;//老六攻击我
		hurt_times_ago++;
				if(ext_robot_hurt.data.hurt_type==0)
		{
			if(ext_robot_hurt.data.armor_id==0||ext_robot_hurt.data.armor_id==1)
			{
				hurt_times_ago=0;//被击中了
			
			if(ext_robot_hurt.data.armor_id==1)//背面装甲板受击
			{
			laoliu_gjiwo_times_ago=0;//老六攻击我
			
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
	const TickType_t TimeIncrement = pdMS_TO_TICKS(2); //每十毫秒强制进入总控制
	/* Infinite loop */
	for (;;)
	{
#if send_way == 0


#endif
		if(DR16.rc.s_left==2)
		{
if(DR16.rc.ch0<-600&&DR16.rc.ch1<-600&&DR16.rc.ch3<-600&&DR16.rc.ch2>600)
{
calibration_times++;

}
else
{
calibration_times=0;

}
if(calibration_times>500)
{
MIT_calibration();
calibration_times=-10000;
	Buzzer.mode = One_times;
}
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
	// CAN2接收队列发送的数据的结构体变量
	CAN_Rx_TypeDef CAN2_Rx_Structure;
	//出队的状态变量
	BaseType_t ExitQueue_Status;
int i=0;
int speed_i=0;

	/* Infinite loop */
	for (;;)
	{
		//死等队列有消息
		ExitQueue_Status = xQueueReceive(CAN2_Queue, &CAN2_Rx_Structure, portMAX_DELAY);
		//出队成功
		if (ExitQueue_Status == pdTRUE)
		{
			CAN2_rc_times++;
			can2_DR16_TIMES++;
			//陀螺仪校准指令
//			if (CAN2_Rx_Structure.CAN_RxMessage.StdId == IMU_CAL_REIID)
//			{
//				//解包
//				IMU_Cal_Status_Reivece(CAN2_Rx_Structure);
//			}
    if(CAN2_Rx_Structure.CAN_RxMessage.StdId == TEST_MIT_MASTER_ID)
	{
	MIT_RC_TIMES++;
	    if(CAN2_Rx_Structure.CAN_RxMessageData[0]== TEST_MIT_SLAVE_ID)
		{
			
			for(int x=0;x<6;x++)
			{
		text_moto.MIT_RAW_DATA[x]=	CAN2_Rx_Structure.CAN_RxMessageData[x];
			}
text_moto.position=(CAN2_Rx_Structure.CAN_RxMessageData[2] | CAN2_Rx_Structure.CAN_RxMessageData[1] << 8);
			
text_moto.velocity=
(((CAN2_Rx_Structure.CAN_RxMessageData[4])>>4 )| 
(CAN2_Rx_Structure.CAN_RxMessageData[3]) << 4);
			
text_moto.current=(CAN2_Rx_Structure.CAN_RxMessageData[5] | (CAN2_Rx_Structure.CAN_RxMessageData[4]&0xF) << 8);

			text_moto.position_end=uint_to_float(text_moto.position,P_MIN,P_MAX,16);
			text_moto.ANGLE_JD=text_moto.position_end* Angle_turn_Radian;
//speed_i++;
//MIT_SPEED_BY_ANGLE_TEMP+=(text_moto.position-MIT_ANGLE_JD_LAST);
//			MIT_ANGLE_JD_LAST_LAST=MIT_ANGLE_JD_LAST;
//MIT_ANGLE_JD_LAST=	text_moto.position;
//			
//			if(speed_i>i_for_speed)
//			{
//				MIT_SPEED_BY_ANGLE=MIT_SPEED_BY_ANGLE_TEMP;
//				MIT_SPEED_BY_ANGLE_TEMP=0;
//				speed_i=0;
//			}
				MIT_SPEED_NEW=LPF_V2(&SPEED_MIT,text_moto.velocity)+5;

			text_moto.velocity_end=uint_to_float(MIT_SPEED_NEW,V_MIN,V_MAX,12);
			text_moto.SPEED_JD=text_moto.velocity_end* Angle_turn_Radian;
			
			text_moto.current_end=uint_to_float(text_moto.current,T_MIN,T_MAX,12);
			
			
		
			MIT_RC_Process_TIMES++;
		}
	}
    if(CAN2_Rx_Structure.CAN_RxMessage.StdId == MIT_A_MASTER_ID)
	{
	    if(CAN2_Rx_Structure.CAN_RxMessageData[0]== MIT_A_SLAVE_ID)
		{
			
			for(int x=0;x<6;x++)
			{
		MIT_A.MIT_RAW_DATA[x]=	CAN2_Rx_Structure.CAN_RxMessageData[x];
			}
MIT_A.position=(CAN2_Rx_Structure.CAN_RxMessageData[2] | CAN2_Rx_Structure.CAN_RxMessageData[1] << 8);
			
MIT_A.velocity=
(((CAN2_Rx_Structure.CAN_RxMessageData[4])>>4 )| 
(CAN2_Rx_Structure.CAN_RxMessageData[3]) << 4);
			
MIT_A.current=(CAN2_Rx_Structure.CAN_RxMessageData[5] | (CAN2_Rx_Structure.CAN_RxMessageData[4]&0xF) << 8);

			MIT_A.position_end=uint_to_float(MIT_A.position,P_MIN,P_MAX,16);
			MIT_A.ANGLE_JD=MIT_A.position_end* Angle_turn_Radian;

				MIT_A.MIT_SPEED_TEMP=LPF_V2(&SPEED_MIT_A,MIT_A.velocity)+5;

			MIT_A.velocity_end=uint_to_float(MIT_A.MIT_SPEED_TEMP,V_MIN,V_MAX,12);
			MIT_A.SPEED_JD=MIT_A.velocity_end* Angle_turn_Radian;
			
			MIT_A.current_end=uint_to_float(MIT_A.current,T_MIN,T_MAX,12);
			
			
		
		}
		MIT_A.RC_TIMES++;
	}

    if(CAN2_Rx_Structure.CAN_RxMessage.StdId == MIT_B_MASTER_ID)
	{
	    if(CAN2_Rx_Structure.CAN_RxMessageData[0]== MIT_B_SLAVE_ID)
		{
			
			for(int x=0;x<6;x++)
			{
		MIT_B.MIT_RAW_DATA[x]=	CAN2_Rx_Structure.CAN_RxMessageData[x];
			}
MIT_B.position=(CAN2_Rx_Structure.CAN_RxMessageData[2] | CAN2_Rx_Structure.CAN_RxMessageData[1] << 8);
			
MIT_B.velocity=
(((CAN2_Rx_Structure.CAN_RxMessageData[4])>>4 )| 
(CAN2_Rx_Structure.CAN_RxMessageData[3]) << 4);
			
MIT_B.current=(CAN2_Rx_Structure.CAN_RxMessageData[5] | (CAN2_Rx_Structure.CAN_RxMessageData[4]&0xF) << 8);

			MIT_B.position_end=uint_to_float(MIT_B.position,P_MIN,P_MAX,16);
			MIT_B.ANGLE_JD=MIT_B.position_end* Angle_turn_Radian;

				MIT_B.MIT_SPEED_TEMP=LPF_V2(&SPEED_MIT_B,MIT_B.velocity)+5;

			MIT_B.velocity_end=uint_to_float(MIT_B.MIT_SPEED_TEMP,V_MIN,V_MAX,12);
			MIT_B.SPEED_JD=MIT_B.velocity_end* Angle_turn_Radian;
			
			MIT_B.current_end=uint_to_float(MIT_B.current,T_MIN,T_MAX,12);
			
			
		
		}
		MIT_B.RC_TIMES++;
	}

    if(CAN2_Rx_Structure.CAN_RxMessage.StdId == MIT_C_MASTER_ID)
	{
	    if(CAN2_Rx_Structure.CAN_RxMessageData[0]== MIT_C_SLAVE_ID)
		{
			
			for(int x=0;x<6;x++)
			{
		MIT_C.MIT_RAW_DATA[x]=	CAN2_Rx_Structure.CAN_RxMessageData[x];
			}
MIT_C.position=(CAN2_Rx_Structure.CAN_RxMessageData[2] | CAN2_Rx_Structure.CAN_RxMessageData[1] << 8);
			
MIT_C.velocity=
(((CAN2_Rx_Structure.CAN_RxMessageData[4])>>4 )| 
(CAN2_Rx_Structure.CAN_RxMessageData[3]) << 4);
			
MIT_C.current=(CAN2_Rx_Structure.CAN_RxMessageData[5] | (CAN2_Rx_Structure.CAN_RxMessageData[4]&0xF) << 8);

			MIT_C.position_end=uint_to_float(MIT_C.position,P_MIN,P_MAX,16);
			MIT_C.ANGLE_JD=MIT_C.position_end* Angle_turn_Radian;

				MIT_C.MIT_SPEED_TEMP=LPF_V2(&SPEED_MIT_C,MIT_C.velocity)+5;

			MIT_C.velocity_end=uint_to_float(MIT_C.MIT_SPEED_TEMP,V_MIN,V_MAX,12);
			MIT_C.SPEED_JD=MIT_C.velocity_end* Angle_turn_Radian;
			
			MIT_C.current_end=uint_to_float(MIT_C.current,T_MIN,T_MAX,12);
			
			
		
		}
		MIT_C.RC_TIMES++;
	}
    if(CAN2_Rx_Structure.CAN_RxMessage.StdId == MIT_D_MASTER_ID)
	{
	    if(CAN2_Rx_Structure.CAN_RxMessageData[0]== MIT_D_SLAVE_ID)
		{
			
			for(int x=0;x<6;x++)
			{
		MIT_D.MIT_RAW_DATA[x]=	CAN2_Rx_Structure.CAN_RxMessageData[x];
			}
MIT_D.position=(CAN2_Rx_Structure.CAN_RxMessageData[2] | CAN2_Rx_Structure.CAN_RxMessageData[1] << 8);
			
MIT_D.velocity=
(((CAN2_Rx_Structure.CAN_RxMessageData[4])>>4 )| 
(CAN2_Rx_Structure.CAN_RxMessageData[3]) << 4);
			
MIT_D.current=(CAN2_Rx_Structure.CAN_RxMessageData[5] | (CAN2_Rx_Structure.CAN_RxMessageData[4]&0xF) << 8);

			MIT_D.position_end=uint_to_float(MIT_D.position,P_MIN,P_MAX,16);
			MIT_D.ANGLE_JD=MIT_D.position_end* Angle_turn_Radian;

				MIT_D.MIT_SPEED_TEMP=LPF_V2(&SPEED_MIT_D,MIT_D.velocity)+5;

			MIT_D.velocity_end=uint_to_float(MIT_D.MIT_SPEED_TEMP,V_MIN,V_MAX,12);
			MIT_D.SPEED_JD=MIT_D.velocity_end* Angle_turn_Radian;
			
			MIT_D.current_end=uint_to_float(MIT_D.current,T_MIN,T_MAX,12);
			
			
		
		}
		MIT_D.RC_TIMES++;
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
	// CAN1接收队列发送的数据的结构体变量
	CAN_Rx_TypeDef CAN1_Rx_Structure;

	//出队的状态变量
	BaseType_t ExitQueue_Status;
	/* Infinite loop */
	for (;;)
	{
		//死等队列有消息
		ExitQueue_Status = xQueueReceive(CAN1_Queue, &CAN1_Rx_Structure, portMAX_DELAY);
		//出队成功

		if (ExitQueue_Status == pdTRUE)
		{
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (M3508_READID_START + 3))
			{
				//左摩擦轮的    ID1
				M3508_getInfo(CAN1_Rx_Structure); //
			}
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (M3508_READID_START + 2))
			{
				//右摩擦轮的	ID2
				M3508_getInfo(CAN1_Rx_Structure); //
			}
			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (M3508_READID_START + 1))
			{
				//拨盘电机		ID3
				M2006_getInfo(CAN1_Rx_Structure); //
			}

			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == GM6020_READID_START)
			{
				//云台
				GM6020_Yaw_getInfo(CAN1_Rx_Structure);
			}

			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == (GM6020_READID_START + 1))
			{
				//云台的yaw轴
				GM6020_Pitch_getInfo(CAN1_Rx_Structure); // 6400-7160
			}

			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == GM6020_READID_START + 2)
			{
				//云台
				GM6020_Yaw_getInfo(CAN1_Rx_Structure);
			}

			if (CAN1_Rx_Structure.CAN_RxMessage.StdId == GM6020_READID_END)
			{
				//云台的pitch轴3743-4557
				GM6020_Pitch_getInfo(CAN1_Rx_Structure); // 6400-7160
			}
		}
	}
  /* USER CODE END CAN1_R */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
uint8_t TRY[20]; //匿名上位机发送数据

int Driver_add = 32768;

void Robot_Control(void const *argument)
{
	/* USER CODE BEGIN RobotControl */
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(1); //每1毫秒强制进入总控制

    vTaskDelay(1000);
    vTaskDelay(1000);

    vTaskDelay(1000);
    vTaskDelay(1000);

	/* Infinite loop */
	for (;;)
	{
	
//		GM6020_SetVoltage(send_to_yaw,0 , 0, send_to_pitch); //云台  send_to_pitch
//		GM6020_SetVoltage(0,0 , 0, 0); //云台  send_to_pitch
		
		balance_control();
	

if(send_to_tire_L>15000)
	send_to_tire_L=15000;
if(send_to_tire_L<-15000)
	send_to_tire_L=-15000;
if(send_to_tire_R>15000)
	send_to_tire_R=15000;
if(send_to_tire_R<-15000)
	send_to_tire_R=-15000;

if(DR16.rc.s_left==2)
{
send_to_tire_R=0;
send_to_tire_L=0;
}

//		M3508s1_setCurrent(0, 0, send_to_tire_R, send_to_tire_L);//send_to_SHOOT_L阻力大
//CAN2_SEND_TO_MIT();
//		M3508s1_setCurrent(0, send_to_2006, send_to_SHOOT_R, send_to_SHOOT_L);//send_to_SHOOT_L阻力大

if(DR16.rc.s_left==2)
{
MIT_MODE_TEXT=2;//电机失能
//MIT_MODE(MIT_MODE_TEXT);
	
DISABLE_ALL_MIT();//失能所有电机
	MIT_A_SPEED.Min_result=MIT_A_SPEED.Max_result=0;
	MIT_B_SPEED.Min_result=MIT_B_SPEED.Max_result=0;
	MIT_C_SPEED.Min_result=MIT_C_SPEED.Max_result=0;
	MIT_D_SPEED.Min_result=MIT_D_SPEED.Max_result=0;
	MIT_OUT.Current_Value=0;
	if(MIT_B.ANGLE_JD<30)liftoff_R=75;
	else liftoff_R=75;
	position_text_TEMP=-1;//重置目标位置
position_text=-1;//重置目标位置
target_position_text_PID=text_moto.ANGLE_JD;
}
else if(DR16.rc.s_left==3)
{
	if(MIT_MODE_TEXT!=1)
	{
MIT_MODE_TEXT=1;
//		MIT_MODE(MIT_MODE_TEXT);//使能电机
		ALL_MIT_ENTER_MOTO_MODE();//使能所有电机
	}
	else
	{
	MIT_controul();
	
	}
	

}
else if(DR16.rc.s_left==1)
{
	if(sendto_MIT_TEXT==1)
			{
		MIT_MODE(MIT_MODE_TEXT);
			sendto_MIT_TEXT=0;
		}

}




vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
		
	}

	/* USER CODE END RobotControl */
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
