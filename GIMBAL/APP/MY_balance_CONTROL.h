#ifndef BALANCE_H
#define BALANCE_H
#include "main.h"

void balance_control(void);
extern float L_R_XS;
extern float total_pitch_change;
extern int M3508_speed_new;
extern int L_speed_new;
extern int R_speed_new;
extern float TARGET_angle_PITCH_MAX;
extern float TARGET_angle_PITCH;
extern int TARGET_position_k;
extern int TARGET_position;
extern float TARGET_angle_PITCH_BC;//»úÐµÆ½ºâ½Ç¶È²¹³¥

#endif


