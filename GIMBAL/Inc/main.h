/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//��������Ĳ���?
//��ջ��С
#define RobotCtrl_Size 512
//���ȼ�
#define RobotCtrl_Priority osPriorityRealtime
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define debug_rx_Pin GPIO_PIN_7
#define debug_rx_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_0
#define OLED_DC_GPIO_Port GPIOF
#define debug_tx_Pin GPIO_PIN_9
#define debug_tx_GPIO_Port GPIOA
#define RED_Pin GPIO_PIN_8
#define RED_GPIO_Port GPIOC
#define OLED_RST_Pin GPIO_PIN_1
#define OLED_RST_GPIO_Port GPIOF
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define Buzzer_Pin GPIO_PIN_14
#define Buzzer_GPIO_Port GPIOD
#define key_Pin GPIO_PIN_0
#define key_GPIO_Port GPIOA
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

//#define OLED_DC_Pin GPIO_PIN_9
//#define OLED_DC_GPIO_Port GPIOB

//#define OLED_RST_Pin GPIO_PIN_10
//#define OLED_RST_GPIO_Port GPIOB
#define SHOOT_HIGH_HEAT_TEXT 0// ������ 
#define use_new_gimbal 1

extern bool ins_ok;
extern int controul_times;
extern int ins_times;
extern int DR16_times;
extern int M2006_times;


extern int send_to_2006;
extern int M2006_targe_speed;

extern int beep_Val;


extern	int	send_to_SHOOT_R;
extern	int	send_to_SHOOT_L;

extern int SHOOT_L_speed;//��Ħ���ֵ�Ŀ���ٶ�   Ӧ��Ϊ��ֵ
extern int SHOOT_R_speed;//��Ħ���ֵ�Ŀ���ٶ�

extern	int	send_to_tire_R;/*����*/
extern	int	send_to_tire_L;/*����*/
extern int tire_L_TARGE_speed;//左轮的目标�?�度   应该为负�??
extern int tire_R_TARGE_speed;//右轮的目标�?�度
extern int tire_L_TARGE_speed_FAKE;//左轮的目标�?�度 (�?)  应该为负�??
extern int tire_R_TARGE_speed_FAKE;//右轮的目标�?�度(�?)
extern int DW_FREE;
extern int DW_DOWN;
extern int SHOOT;
extern int Driver_add;

extern int yaw_trage_speed;
extern float yaw_trage_angle;
extern float yaw_trage_angle_1s_ago;
extern float yaw_trage_angle_add_1s;


extern int send_to_yaw;
extern int send_to_pitch;//���͸�pitch�������?


extern float PITCH_MAX_angle;
extern float PITCH_MIN_angle;
extern float allow_angle;//�ɶ�����

extern float PITCH_trage_angle;
extern float PITCH_trage_angle_motor;

extern int PITCH_trage_speed;
extern int can1_IT_TIMES;

extern int can2_DR16_TIMES;
extern int can2_IT_TIMES;
extern int CH0_TOTAL;
extern int CH1_TOTAL;
extern int CH2_TOTAL;
extern int CH3_TOTAL;

extern int JS_RC_TIMES;
extern bool run_JS_jiema;

extern int STATUS_PART_ONE_TIMES;
extern int STATUS_PART_TWO_TIMES;
extern int STATUS_PART_THREE_TIMES;
extern int STATUS_PART_FOUR_TIMES;
extern int STATUS_complete_update_TIMES;
extern int HEAT_complete_update_TIMES;//����ϵͳ������Ϣ���³ɹ�����
extern int place_complete_update_TIMES;//����ϵͳ������Ϣ���³ɹ�����

extern int HP_complete_update_TIMES;//����ϵͳ״̬��Ϣ���³ɹ�����
extern int GAME_STATE_update_TIMES;//����ϵͳ����״̬��Ϣ���³ɹ�����

extern int DDR16_PART_ONE_TIMES;
extern int DDR16_PART_TWO_TIMES;
extern int DDR16_PART_THREE_TIMES;
extern int dr16_controul_times;
extern int CH0_TOTAL_in_con;
extern bool run_DR16_jiema;
extern float Vision_RawData_Yaw_Angle;
extern float Vision_RawData_Pitch_Angle;
void ConfigureTimeForRunTimeStats(void);

extern int TEST_Current_L;
extern int TEST_Current_R;

extern int CLOUD_enable_moto;
extern float CLOUD_enable_imu;
extern bool cloud_text_add;
extern bool cloud_enable;
extern int VISION_FROM_USART1;
extern int VISION_Disconnect_test;

extern int Armour_lose_time;
extern int SHOOT_STOP_time;

extern int vision_shoot_times;

extern bool disable_for_test;
extern int shoot_times_for_limit;
extern bool whether_shoot_in__this_period;//��������Ƿ���?
extern bool this_period_has_shoot;//��������Ƿ���?
extern int this_period_has_shoot_number;//������ڷ���shu
extern int every_shoot_number;//һ����ʮ��
extern int targe_shoot_number;//һ����ʮ��
extern int shoot_speed_text;

extern int ch4_DW_total;
extern int ch4_DW_total_2;
extern bool crc_right;
extern int vision_rc_right;
extern int vision_rc_error;
extern int CAN2_rc_times;

extern float simulation_target_yaw;

extern bool TEMPERATURE_is_OK;
extern int TEMPERATURE_PID_OUT;

extern bool in_MID;//���ڹ���м��
extern bool in_END;//���ڹ�����?
extern bool in_END_R;//�����ҹ�����?
extern bool in_END_L;//����������ͷ
extern bool in_END_last;//��һʱ�̴��ڹ�����?

extern bool disable_for_test_CHASSIS;
extern float my_voltage;

extern int stay_in_track_end_times;

extern bool send_to_vision_1;
 extern int text_times;

 extern int hurt_times_ago;
 extern int laoliu_gjiwo_times_ago;//����������

extern float jia_ZJ_YAW;
extern float jia_ZJ_PITCH;
 extern bool vision_beats_give_to_jia;

/*��װ��Ҫ�õ�GEBIN*/
extern  int8_t whether_use_fake_armor;
 extern int fake_armor_init_angle_6020;
extern int fake_armor_init_place_encoder;
 extern int fake_armor_vertical_place_encoder;

extern int fake_armor_init_vision_deepth;/*mm*/
extern int fake_armor_vertica_distance;/*��ֱ����mm*/
extern float fake_armor_now_angle_IMU;
extern float YAW_TRAGET_ANGLE_TEMP_FAKE_MOTO;
extern float YAW_TRAGET_ANGLE_TEMP_FAKE_IMU;

extern float send_to_pitch_before;


extern int MIT_RC_TIMES;
extern int MIT_RC_Process_TIMES;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
