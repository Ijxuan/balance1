#ifndef BEEP_H
#define BEEP_H


#include "main.h"
#include "tim.h"
#include "bsp_buzzer.h"
#include "sound_effects_task.h"
#include "bsp_buzzer.h"


#define KEY_down 0
#define KEY_up 1

extern int beep_val;

extern bool change_beel;
extern int  fen_p;//��Ƶ
extern int  aoto_period;//�Զ���װ��
extern int  Compare;//�Ƚ�ֵ
extern int  YS;//Ԥ����
extern bool zxyici;

void BEEP_TEXT(void);



#endif


