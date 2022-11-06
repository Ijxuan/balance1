/**
  *************************(C) COPYRIGHT 2020 LionHeart*************************
  * @file       bsp_buzzer_driver.c/h
  * @brief      �����������������������ֲ�����������Ҫ��д���������Ĵ��롣��ϸ
  *             �������������RoboMaster�ٷ���GitHub��Դ�ĵ�����RoboMaster������C��
  *             Ƕ��ʽ����̳��ĵ�.pdf����71ҳ��
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Sep-19-2020     LionHeart       1. done
  *
  @verbatim
  ==============================================================================
  ������
	���������Ͷ��壺struct_typedef.h
  ==============================================================================
  @endverbatim
  *************************(C) COPYRIGHT 2020 LionHeart*************************
  */

#ifndef __BSP_BUZZER_DRIVER_H
#define __BSP_BUZZER_DRIVER_H

#include "struct_typedef.h"


/**
 * @brief          ���Ʒ�������ʱ���ķ�Ƶ������ֵ
 * @param[in]      psc�����ö�ʱ���ķ�Ƶϵ��
 * @param[in]      pwm�����ö�ʱ��������ֵ
 * @retval         none
 */
extern void buzzer_drv_on(uint16_t psc, uint16_t pwm);

/**
  * @brief          �رշ�����
  * @param[in]      none
  * @retval         none
  */
extern void buzzer_drv_off(void);

#endif
