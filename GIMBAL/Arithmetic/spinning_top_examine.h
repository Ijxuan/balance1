#ifndef S_T_examine_H
#define S_T_examine_H
#include "DR16_RECIVE.h"



void S_T_examine(void);
void fake_armor_yaw(void);


extern float total_yaw_change;//�仯�ĽǶ�
extern int examine_sampling_period;//С���ݼ�������=20ms,�������ý���һ�β���
extern int8_t YAW_MOTION_STATE;//С���ݼ��״̬

extern float temp_angle;
extern float temp_rus;
extern float SJX_D;

#endif

