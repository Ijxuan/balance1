#include"BEEP_MY.h"


int beep_val=0;


//�ı�Ƶ��: 83*10^6  / ����Ƶϵ�� * �Զ���װ��ֵ�� 
/*
���磺      84 000 000  /��1*21000��=40000
            84 000 000  / 80,305     =1046
                         8,030*10   			
			84 000 000  / 71,489     =1175 
						7,149*10
            84 000 000  / 63,733    =1318
			            6,373*10
//1 - do - 1046Hz
//2 - re - 1175Hz
//3 - mi - 1318Hz
  
*/

//�ı�ռ�ձȣ�  �Ƚ�ֵ  /  �Զ���װ��ֵ
/**
  * @brief          ���Ʒ�������ʱ���ķ�Ƶ������ֵ
  * @param[in]      psc�����ö�ʱ���ķ�Ƶϵ��
  * @param[in]      pwm�����ö�ʱ��������ֵ
  * @retval         none
  */
bool change_beel=0;
int  fen_p=0;//��Ƶ
int  aoto_period=0;//�Զ���װ��
int  Compare=0;//�Ƚ�ֵ
int  YS=0;//Ԥ����
bool zxyici=0;
void BEEP_TEXT()
{
//			static uint8_t KEY_last;		//key�ϴεĵ�ƽ
//					uint8_t KEY_NOW;		//key�ϴεĵ�ƽ
//		KEY_NOW = HAL_GPIO_ReadPin(key_GPIO_Port,key_Pin);
	if(change_beel==1)
	{
		
		if(YS==1)//������Ͽ
		{
			fen_p=1;
			aoto_period=65535;
			Compare=10000;
			YS=0;
		}
		if(YS==2)//1 - do - 1046Hz
		{
			fen_p=9;
			aoto_period=8030;
			Compare=4015;
			YS=0;
		}
		if(YS==3)//2 - re - 1175Hz
		{
			fen_p=9;
			aoto_period=7149;
			Compare=3575;
			YS=0;
		}
		if(YS==4)//3 - mi - 1318Hz
		{
			fen_p=9;
			aoto_period=6373;
			Compare=3186;
			YS=0;
		}		
		
		
		
//		htim4.Init.Prescaler = fen_p;//���ö�ʱ���ķ�Ƶϵ��
//		htim4.Init.Period = aoto_period;//���ö�ʱ��������ֵ
//		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, Compare);//���ñȽ�ֵ

		change_beel=0;
	}
	
	
//	if(KEY_NOW == KEY_down && KEY_last == KEY_up)
//		//���²Ż�����������ס���Ų��У�
//	{
////						buzzer_control.sound_effect=B_;
////		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, Compare);//�е�

//	}
	
//	if(KEY_NOW== KEY_up)
//	{
////		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);//�е�

//		
//	}
	
//	if(KEY_NOW== KEY_down)
//	{

//		
//	}
	
	if(zxyici==1)
	{
//				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, Compare);//�е�
//HAL_Delay(1000);
//		HAL_Delay(1000);
//				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);//�е�

//		zxyici=0;
	}
//	Buzzer.buzzer_value=0;
	
//	KEY_last=KEY_NOW;
	
//	  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, Buzzer.buzzer_value); 
	
	
	
	
	
	
	
	
	
	
	
	
	
			if (0)
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
				buzzer_drv_on(1, 10000);	HAL_Delay(1000);
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
//			buzzer_control.sound_effect = STOP;
//			buzzer_drv_off();
//			buzzer_is_busy = FALSE;
		}
	
	
	
	
	
	
	
	
	
	
	
	

	
	
}























