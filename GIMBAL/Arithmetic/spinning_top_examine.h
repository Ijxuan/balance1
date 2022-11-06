#ifndef S_T_examine_H
#define S_T_examine_H
#include "DR16_RECIVE.h"



void S_T_examine(void);
void fake_armor_yaw(void);


extern float total_yaw_change;//变化的角度
extern int examine_sampling_period;//小陀螺检测的周期=20ms,即间隔多久进行一次采样
extern int8_t YAW_MOTION_STATE;//小陀螺检测状态

extern float temp_angle;
extern float temp_rus;
extern float SJX_D;

#endif

