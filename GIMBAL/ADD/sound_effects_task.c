/**
  *************************(C) COPYRIGHT 2020 LionHeart*************************
  * @file       sound_effects.c/h
  * @brief      ʵ�ַ���������Ч�����Ĺ��ܡ�����������12�ַ�����Ч�����������û���
  *             �����á��ڻ����˿�/�ع��ܡ�״̬�л���ʶ��Ŀ���ʱ�����Ӧ��Ч������
  *             �����ճ��з��и���������Աʶ�������״̬���жϳ�������н��ȣ��Լ�
  *             �����������и�����Ա���ٵ��������ˡ�
  *             �����������RoboMaster-C�Ϳ�����İ��ط������������ࡢ��㣬ռ��
  *             оƬ��Դ���٣��ʺ����ڸ��ֿ������ϡ�
  *
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Sep-19-2020     LionHeart       1. done
  *
  @verbatim
  ==============================================================================
  ʹ��˵����

	1.����ά����
		ʹ��FreeRTOSά����������buzzer_effects_task(void const *argument)����֤
		������õ��ϸߵ����ȼ���
		��Ҫ����ͷ�ļ���#include "sound_effects_task.h" ����������������

	2.���ܵ��ã���������
		��������һԴ�ļ���xxx.c"����Ҫ������������Ч����������Ҫ����ͷ�ļ���
			#include "sound_effects_task.h"
		Ȼ����Ҫ����һָ�룬ʹ��ָ��sound_effects_task�е�buzzer_control���˴�ֱ��
		���ú��� get_buzzer_effect_point() ����ȡ��ַ�����磺
			buzzer_t *buzzer = get_buzzer_effect_point();
		����Ҫ�ڳ�����е�ĳ�׶�ʱ������ָ������Ч����ֱ�Ӳ���ָ���sound_effect��
		�ɡ��Դ�����ϵͳ������Ч��Ϊ����
			......
			buzzer->sound_effect = SYSTEM_START_BEEP;
			......
		��
			......
			if (*(buzzer->is_busy) == FALSE)
				buzzer->sound_effect = SYSTEM_START_BEEP;
			......
		�����������е��÷�������������������ͻ�����·�������Ч����������ͨ��������
			buzzer->work = FALSE;
		��ͣ�÷�������Ч��������ʱ���������Ի���ɵ�ǰ�����������Ч��Ȼ��Ż�ֹͣ��
		�йظ�����Ч��˵�������sound_effects_task.h�е�sound_effects_tö�����͡�

	3.�������ã�
		�����򻹰���buzzer_TIM_init.c/h��bsp_buzzer_driver.c/h�����ļ�����Щ�ļ���
		��ֲ��RM2020�ٷ�������Դ���򡣱���������HAL�⡰stm32f4xx_hal.c/h����������ȷ
		����ֲ��Ŀ�깤��ʹ��HAL�⣬������Ҫ�������buzzer_init.c/h��bsp_buzzer.c/h
		�еĲ��ֺ�����Ҳ����HALֱ�Ӳ�����ʼ�����룬��ϸ�������������RoboMaster�ٷ�
		��GitHub��Դ�ĵ�����RoboMaster������C��Ƕ��ʽ����̳��ĵ�.pdf����71ҳ��
		��RM2020�ٷ���Դ����Ϊ������Ҫ���˳�����ֲ��RM2020�ٷ�������������Ҫ��
			����Դ�ļ���ͷ�ļ����Ƶ�����Ŀ¼�£����ڹ��������Դ�ļ�
			����freertos.c�а���ͷ�ļ���#include "sound_effects_task.h"
			��������������Ч����
				osThreadDef(buzr, buzzer_effects_task, osPriorityNormal, 0, 128);
				testHandle = osThreadCreate(osThread(buzr), NULL);
			����test_task.c������ͷ�ļ������������к��ʵ�λ������ӣ�
				buzzer_t *buzzer = get_buzzer_effect_point();
				......
				buzzer->work = FALSE;
				......
				buzzer->work = TRUE;
			 �������ٷ������е�ģ��������ʾ���������ͻ��ɵ���Ч�쳣����ʵ������Ҳ
			 û�д����⣬ֻ����������һ����ѣ�
			����������������Դ�ļ���ִ�й��ܵ��ã����������衣
		��ֲ�����������У�����Ҫ���ݾ��������������������
  ==============================================================================
  @endverbatim
  *************************(C) COPYRIGHT 2020 LionHeart*************************
  */

#include "sound_effects_task.h"

buzzer_t buzzer_control;
bool_check_t buzzer_is_busy;

/**
  * @brief          ����buzzer_is_busyָ��
  * @param[in]      none
  * @retval         bool_check_t *buzzer_is_busy
  */
const bool_check_t *get_buzzer_is_busy_point(void);


/**
  * @brief          ��������Ч���񣬼�� BUZZER_TASK_CONTROL_TIME 10ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void buzzer_effects_task(void const *argument)
{
	//��ʼ����־λ
	buzzer_control.is_busy = get_buzzer_is_busy_point();
	buzzer_is_busy = FALSE;   
	buzzer_control.work = TRUE; 
	buzzer_control.sound_effect = STOP;  

	//�ȴ�����������ģ���ʼ�����
	osDelay(500);
	//��ʼ��TIM4��Ϊ����������
	MXY_TIM4_Init();
	//�رշ�����
	buzzer_drv_off();
	//����һ�Ρ�������������Ч��������Ҫ�ڴ�ʹ�ã��뽫����ע��
	buzzer_control.sound_effect = SYSTEM_START_BEEP;

	for (;;)
	{
		//��鹤����־�Ƿ���λ
		if (buzzer_control.work == TRUE)
		{
			switch (buzzer_control.sound_effect)
			{
			case STOP:
				buzzer_drv_off();
				buzzer_is_busy = FALSE;
				break;

			case SYSTEM_START_BEEP:
				buzzer_control.sound_effect = STOP;
				buzzer_is_busy = TRUE;
				buzzer_drv_on(3, 10000);	osDelay(333);
				buzzer_drv_on(2, 10000);	osDelay(333);
				buzzer_drv_on(1, 10000);	osDelay(333);
				buzzer_drv_off();
				buzzer_is_busy = FALSE;
				break;

			case B_:
				buzzer_control.sound_effect = STOP;
				buzzer_is_busy = TRUE;
				buzzer_drv_on(1, 10000);	osDelay(70);
				buzzer_drv_off();
				buzzer_is_busy = FALSE;
				break;

			case B_B_:
				buzzer_control.sound_effect = STOP;
				buzzer_is_busy = TRUE;
				buzzer_drv_on(1, 10000);	osDelay(70);
				buzzer_drv_off();			osDelay(70);
				buzzer_drv_on(1, 10000);	osDelay(70);
				buzzer_drv_off();
				buzzer_is_busy = FALSE;
				break;

			case B_B_B_:
				buzzer_control.sound_effect = STOP;
				buzzer_is_busy = TRUE;
				buzzer_drv_on(1, 10000);	osDelay(70);
				buzzer_drv_off();			osDelay(70);
				buzzer_drv_on(1, 10000);	osDelay(70);
				buzzer_drv_off();			osDelay(70);
				buzzer_drv_on(1, 10000);	osDelay(70);
				buzzer_drv_off();
				buzzer_is_busy = FALSE;
				break;

			case B___:
				buzzer_control.sound_effect = STOP;
				buzzer_is_busy = TRUE;
				buzzer_drv_on(1, 10000);	osDelay(500);
				buzzer_is_busy = FALSE;
				break;

			case B_CONTINUE:
				buzzer_is_busy = TRUE;
				buzzer_drv_on(1, 10000);	osDelay(100);
				buzzer_drv_off();           osDelay(50);
				break;

			case D_:
				buzzer_control.sound_effect = STOP;
				buzzer_is_busy = TRUE;
				buzzer_drv_on(4, 10000);	osDelay(70);
				buzzer_is_busy = FALSE;
				break;

			case D_D_:
				buzzer_control.sound_effect = STOP;
				buzzer_is_busy = TRUE;
				buzzer_drv_on(4, 10000);	osDelay(70);
				buzzer_drv_off();			osDelay(70);
				buzzer_drv_on(4, 10000);	osDelay(70);
				buzzer_drv_off();
				buzzer_is_busy = FALSE;
				break;

			case D_D_D_:
				buzzer_control.sound_effect = STOP;
				buzzer_is_busy = TRUE;
				buzzer_drv_on(4, 10000);	osDelay(70);
				buzzer_drv_off();			osDelay(70);
				buzzer_drv_on(4, 10000);	osDelay(70);
				buzzer_drv_off();			osDelay(70);
				buzzer_drv_on(4, 10000);	osDelay(70);
				buzzer_drv_off();
				buzzer_is_busy = FALSE;
				break;

			case D___:
				buzzer_control.sound_effect = STOP;
				buzzer_is_busy = TRUE;
				buzzer_drv_on(4, 10000);    osDelay(500);
				buzzer_is_busy = FALSE;
				break;

			case D_CONTINUE:
				buzzer_is_busy = TRUE;
				buzzer_drv_on(4, 10000);    osDelay(100);
				buzzer_drv_off();           osDelay(50);
				break;

			case D_B_B_:
				buzzer_control.sound_effect = STOP;
				buzzer_is_busy = TRUE;
				buzzer_drv_on(4, 10000);	osDelay(70);
				buzzer_drv_off();			osDelay(70);
				buzzer_drv_on(1, 10000);	osDelay(70);
				buzzer_drv_off();			osDelay(70);
				buzzer_drv_on(1, 10000);	osDelay(70);
				buzzer_drv_off();
				buzzer_is_busy = FALSE;
				break;

			default:
				buzzer_control.sound_effect = STOP;
				buzzer_drv_off();
				buzzer_is_busy = FALSE;
				break;
			}
		}
		else if (buzzer_control.sound_effect != STOP)
		{
			buzzer_control.sound_effect = STOP;
			buzzer_drv_off();
			buzzer_is_busy = FALSE;
		}
		osDelay(BUZZER_TASK_CONTROL_TIME);  //������10ms
	}
}


/**
  * @brief          ���ط�������������ָ��
  * @param[in]      none
  * @retval         buzzer_t *buzzer_control
  */
buzzer_t *get_buzzer_effect_point(void)
{
	return &buzzer_control;
}

/**
  * @brief          ����buzzer_is_busyָ��
  * @param[in]      none
  * @retval         bool_check_t *buzzer_is_busy
  */
const bool_check_t *get_buzzer_is_busy_point(void)
{
	return &buzzer_is_busy;
}
