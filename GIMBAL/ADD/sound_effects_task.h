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

#ifndef __SOUND_EFFECTS_TASK_H
#define __SOUND_EFFECTS_TASK_H
#ifdef __cplusplus
extern "C" {
#endif

#include "buzzer_TIM_init.h"
#include "bsp_buzzer_driver.h"
#include "cmsis_os.h"

//��������Ч��������10ms�����鲻Ҫ����30��
#define BUZZER_TASK_CONTROL_TIME 10

// ----- ��������Ч��ע���û����Ե��õĸ�����Ч������µ���Ч���ڴ�����ö�ٳ�Ա
typedef enum
{
	STOP = 0,           //ֹͣ�������������졣
	SYSTEM_START_BEEP,  //������DJI��Ʒ���������η��������������������ʱʹ��
	B_,                 //һ���̴ٵĸ�����     ��������������ʱʹ�ã���������ϵ�
	B_B_,               //�����̴ٵĸ�����     ������״̬�л�ʱʹ�ã����翪��/�ر�С����
	B_B_B_,             //�����̴ٵĸ�����     ��������Ҫ���ܿ���ʱʹ�ã����翪����������
	B___,               //һ���Ƴ��ĸ�����     ��������Ҫ��������ִ��ʱʹ�ã������Ӿ��Ѿ������׼
	B_CONTINUE,         //�����̴ٵĸ�����     ��������Ҫ��������ִ��ʱʹ�ã������Ӿ������Զ���׼
	D_,                 //һ���̴ٵĵ�����     ��������������ʱʹ�ã���������µ�
	D_D_,               //�����̴ٵĵ�����     �����������ϵͳ����������ʶ��RFID��ǩ
	D_D_D_,             //�����̴ٵĵ�����     ����������ʧ��ʱʹ�ã����繤��δ������Զ���λ
	D___,               //һ���Ƴ��ĵ�����     ��������Ҫ���ܹر�ʱʹ�ã�����رշ�������
	D_CONTINUE,         //�����̴ٵĵ�����     ����������/״̬�쳣ʱʹ�ã����糬����/��������Ѫ
	D_B_B_              //һ��������������������
}sound_effects_t;

//����һ���򵥵Ĳ�����������
typedef enum
{
	FALSE = 0x00U,
	TRUE = 0x01U
}bool_check_t;

//��������������ӿڵ���������
typedef struct
{
	const bool_check_t *is_busy;    //��������æ��־��ֻ����ΪTRUEʱ˵����������������
	bool_check_t work;              //����������ʹ�ܣ����������������û������ͻ����Ҫ��ʱͣ�÷�������Ч����ʱ������дFALSE����
	sound_effects_t sound_effect;   //��������Чʹ�ܣ�����д��������Чö�ٳ�Ա��������Ч
}buzzer_t;


/**
  * @brief          ���ط�������������ָ��
  * @param[in]      none
  * @retval         buzzer_t ��������������ָ��
  */
extern void buzzer_effects_task(void const *argument);


/**
  * @brief          ��������Ч���񣬼�� BUZZER_TASK_CONTROL_TIME 10ms
  * @param[in]      pvParameters: ��
  * @retval         none
  */
extern buzzer_t *get_buzzer_effect_point(void);
////////////////////////////////////////
extern buzzer_t buzzer_control;
extern bool_check_t buzzer_is_busy;
///////////////////////////////////////
#ifdef __cplusplus
}
#endif
#endif /*__SOUND_EFFECTS_TASK_H */

