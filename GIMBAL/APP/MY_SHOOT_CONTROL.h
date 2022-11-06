#ifndef SHOOT_CAN_H
#define SHOOT_CAN_H
#include "main.h"

void shoot_control(void);
void shoot_control_V2(void);

void driver_plate_control(void);
void which_is_gimbal_shoot_ID(void);

typedef struct
{
bool vision_shoot_is_continuous;//�Ӿ�����ָ����������	
bool  heat_allow;//��������
bool  not_in_track_end;//���ڹ��ĩ��
bool  weather_angle_error_less_than_1;//�Ƕ����С��һ
	
bool ALL_condition_satisfaction;//ȫ������������
} allow_auto_shoot;
extern allow_auto_shoot auto_shoot_condition;

extern bool Driver_arrive;
extern int if_Driver_arrive_angle;
extern long M2006_targe_angle;
extern bool weather_error_less_than_1;
extern int auto_shoot_condition_show;//�Զ��������չ��

#endif

