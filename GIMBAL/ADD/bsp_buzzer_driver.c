/**
  *************************(C) COPYRIGHT 2020 LionHeart*************************
  * @file       bsp_buzzer_driver.c/h
  * @brief      �����������������������ֲ�������Ҫ��д���������Ĵ��롣��ϸ����
  *             ���������RoboMaster�ٷ���GitHub��Դ�ĵ�����RoboMaster������C��Ƕ��
  *             ʽ����̳��ĵ�.pdf����71ҳ��
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Sep-19-2020     LionHeart       1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *************************(C) COPYRIGHT 2020 LionHeart*************************
  */


#include "buzzer_TIM_init.h"
#include "stm32f4xx_hal.h"
#include "tim.h"

extern TIM_HandleTypeDef re_htim4;

/**
  * @brief          ���Ʒ�������ʱ���ķ�Ƶ������ֵ
  * @param[in]      psc�����ö�ʱ���ķ�Ƶϵ��
  * @param[in]      pwm�����ö�ʱ��������ֵ
  * @retval         none
  */
void buzzer_drv_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim4, psc);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);

}

/**
  * @brief          �رշ�����
  * @param[in]      none
  * @retval         none
  */
void buzzer_drv_off(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}
