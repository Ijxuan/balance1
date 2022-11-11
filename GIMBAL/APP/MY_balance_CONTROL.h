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

#endif


