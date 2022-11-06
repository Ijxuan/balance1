/**
  *************************(C) COPYRIGHT 2020 LionHeart*************************
  * @file       buzzer_TIM_init.c/h
  * @brief      RoboMaster-C�Ϳ������������IO��TIM��ʼ����Դ����ֲ�Թٷ�2020����
  *             ��Դ��������ֲ�����������Ҫ��д���������Ĵ��롣��ϸ�����������
  *             ��RoboMaster�ٷ���GitHub��Դ�ĵ�����RoboMaster������C��Ƕ��ʽ�����
  *             ���ĵ�.pdf����71ҳ��
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Sep-19-2020     LionHeart       1. done
  *
  @verbatim
  ==============================================================================
  ������
    ��HAL�⣺stm32f4xx_hal.h
  ==============================================================================
  @endverbatim
  *************************(C) COPYRIGHT 2020 LionHeart*************************
  */

#ifndef __BUZZER_TIM_INIT_H
#define __BUZZER_TIM_INIT_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"

//RM-C���Ϸ��������ӵĽӿ�
#define BUZZER_Pin        GPIO_PIN_14
#define BUZZER_GPIO_Port  GPIOD

//Ϊ������CubeMX�Զ����ɵĴ���������������һ������
extern TIM_HandleTypeDef re_htim4;

/**
  * @brief          ��ֲ��HAL�⣺MX_TIM4_Init(void);
  * @param[in]      none
  * @retval         none
  */
void MXY_TIM4_Init(void);
    
/**
  * @brief          ��ֲ��CubeMX�Զ����ɴ��룺HAL_TIM_MspPostInit(TIM_HandleTypeDef *timHandle);
  * @param[in]      none
  * @retval         none
  */
void TIM_MspPostInit_buzzer(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif
#endif 
