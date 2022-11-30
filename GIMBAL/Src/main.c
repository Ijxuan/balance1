/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_delay.h"

#include "calibrate_task.h"
#include "INS_task.h"
#include "led_flow_task.h"
#include "user_can.h"
#include "bsp_buzzer.h"
#include "stdbool.h"
#include "DR16_RECIVE.h"
#include "oled.h"
#include "bsp_adc.h"
#include "stdio.h"

//#include "oledfont.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool ins_ok=0;
int controul_times=0;
int ins_times=0;
int DR16_times=0;
int M2006_times=0;


int send_to_2006=0;
int M2006_targe_speed=0;
int beep_Val=0;

	int	send_to_SHOOT_R=0;
	int	send_to_SHOOT_L=0;
	int	send_to_tire_R=0;/*右轮*/
	int	send_to_tire_L=0;/*左轮*/
	
int SHOOT_L_speed=0;//左摩擦轮的目标速度   应该为负值
int SHOOT_R_speed=0;//右摩擦轮的目标速度


int tire_L_TARGE_speed=0;//左轮的目标速度   应该为负值?
int tire_R_TARGE_speed=0;//右轮的目标速度

int tire_L_TARGE_speed_FAKE=0;//左轮的目标速度 (假)  应该为负值?
int tire_R_TARGE_speed_FAKE=0;//右轮的目标速度

int yaw_trage_speed=0;
float yaw_trage_angle=0;
float yaw_trage_angle_1s_ago=0;
float yaw_trage_angle_add_1s=0;

int send_to_yaw=0;

int send_to_pitch=0;//发送给pitch轴的数据

float PITCH_MAX_angle=0;
float PITCH_MIN_angle=0;
float allow_angle=0;//可动区间

float PITCH_trage_angle=0;
float PITCH_trage_angle_motor=4500.0;

int PITCH_trage_speed=0;

int can1_IT_TIMES=0;

int can2_DR16_TIMES=0;
int can2_IT_TIMES=0;
int CH0_TOTAL=0;
int CH0_TOTAL_in_con=0;

int CH1_TOTAL=0;
int CH2_TOTAL=0;
int CH3_TOTAL=0;

int JS_RC_TIMES=0;
bool run_JS_jiema;

int STATUS_PART_ONE_TIMES=0;
int STATUS_PART_TWO_TIMES=0;
int STATUS_PART_THREE_TIMES=0;
int STATUS_PART_FOUR_TIMES=0;
int STATUS_complete_update_TIMES=0;//裁判系统状态信息更新成功次数

int HP_complete_update_TIMES=0;//裁判系统状态信息更新成功次数

int HEAT_complete_update_TIMES=0;//裁判系统热量信息更新成功次数
int place_complete_update_TIMES=0;//裁判系统热量信息更新成功次数

int GAME_STATE_update_TIMES=0;//裁判系统比赛状态信息更新成功次数

int DDR16_PART_ONE_TIMES=0;
int DDR16_PART_TWO_TIMES=0;
int DDR16_PART_THREE_TIMES=0;
int dr16_controul_times=0;
bool run_DR16_jiema;

float Vision_RawData_Yaw_Angle=0;
float Vision_RawData_Pitch_Angle=0;
volatile unsigned long long FreeRTOSRunTimeTicks;

int TEST_Current_L=0;
int TEST_Current_R=0;

int CLOUD_enable_moto=0;
float CLOUD_enable_imu=0;
bool cloud_text_add=1;
bool cloud_enable=0;
int VISION_FROM_USART1=0;
int VISION_Disconnect_test=0;
int Armour_lose_time=0;
int SHOOT_STOP_time=0;

int vision_shoot_times;
int shoot_times_for_limit=0;//发射周期(为了热量限制)
bool whether_shoot_in__this_period=0;//这个周期是否发射
bool this_period_has_shoot=0;//这个周期是否发射

int this_period_has_shoot_number=0;//这个周期发射shu
int every_shoot_number=600;//一次三十发
int targe_shoot_number=0;//一次三十发

bool disable_for_test=0;//为了调试
int ch4_DW_total=0;
int ch4_DW_total_2=0;

bool crc_right=0;
int vision_rc_right=0;
int vision_rc_error=0;

int CAN2_rc_times=0;

float simulation_target_yaw=0;

bool TEMPERATURE_is_OK=0;
int TEMPERATURE_PID_OUT=0;

bool in_MID=1;//处在轨道中间段
bool in_END=0;//处在轨道尽头
bool in_END_last=0;//上一时刻处在轨道尽头

bool in_END_R=0;//处在右轨道尽头
bool in_END_L=0;//处在左轨道尽头

bool disable_for_test_CHASSIS=0;

float my_voltage;

int stay_in_track_end_times=0;
int text_times=0;
bool send_to_vision_1=0;

int hurt_times_ago=100000;
int laoliu_gjiwo_times_ago=100000;//老六攻击我
float jia_ZJ_YAW=0;
float jia_ZJ_PITCH=-40.0;
bool vision_beats_give_to_jia=0;

/*假装甲要用到GEBIN*/
 int8_t whether_use_fake_armor=0;
int fake_armor_init_angle_6020=0;
int fake_armor_init_place_encoder=0;
int fake_armor_vertical_place_encoder=0;

int fake_armor_init_vision_deepth=0;/*mm*/
int fake_armor_vertica_distance=0;/*垂直距离mm*/

float fake_armor_now_angle_IMU=0.0;

float YAW_TRAGET_ANGLE_TEMP_FAKE_MOTO;
float YAW_TRAGET_ANGLE_TEMP_FAKE_IMU;

float send_to_pitch_before=0;
/*假装甲要用到END*/

int MIT_RC_TIMES=0;
int MIT_RC_Process_TIMES=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_I2C3_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_CAN2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
	  delay_init();
		Buzzer_Init();
//					Buzzer.status = ON;

    cali_param_init();
   usart1_dr16_init();
   #ifdef FREERTOS_TASK_TIME
       HAL_TIM_Base_Start_IT(&htim3);
#endif
  oled_init(); 
  HAL_Delay(500);

    oled_clear(Pen_Clear);
    oled_LOGO();
    oled_refresh_gram();

    //use vrefint voltage to calibrate
    //使用基准电压来校准
    init_vrefint_reciprocal();
	
//CAN1_Config();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//初始化TIM3使其为FreeRTOS的时间统计提供时基
void ConfigureTimeForRunTimeStats(void)
{
	//定时器3初始化，定时器时钟为84M，分频系数为84-1，所以定时器3的频率
	//为84M/84=1M，自动重装载为50-1，那么定时器周期就是50us
	FreeRTOSRunTimeTicks=0;
//	TIM3_Int_Init(50-1,84-1);	//初始化TIM3
}


//int fputc(int ch, FILE *f)
//{
// uint8_t temp[1] = {ch};
// HAL_UART_Transmit(&huart1, temp, 1, 100);
// return ch;
//}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
