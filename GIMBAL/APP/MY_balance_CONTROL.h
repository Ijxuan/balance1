#ifndef BALANCE_H
#define BALANCE_H
#include "main.h"


typedef struct
{
  int total_mile_by_turnCount;//通过圈数计算里程
  int total_mile_by_angle;//通过totalangle计算里程
	int total_mile_by_angle_100;//通过totalangle除以100计算里程
  int total_mile_by_angle_1000;//通过totalangle除以1000计算里程
  int total_mile_by_angle_8191;//通过totalangle除以8191计算里程
  int total_mile_by_angle_4000;//通过totalangle除以4000计算里程
  int total_mile_truly_use;//真正在用的里程计
}milemeter_t;

void balance_control(void);
void milemeter(void);//里程计函数
void PIRCH_XR(void);//PITCH轴削弱

extern float L_R_XS;
extern float total_pitch_change;
extern int M3508_speed_new;
extern int L_speed_new;
extern int R_speed_new;
extern float TARGET_angle_PITCH_MAX;
extern float TARGET_angle_PITCH;
extern int TARGET_position_k;
extern int TARGET_position;
extern float TARGET_angle_PITCH_BC;//��еƽ��ǶȲ���
extern milemeter_t milemeter_test;
extern float PITCH_ZDJD;//最低角度
extern float PITCH_XR_K;//PITCH轴削弱系数

#endif


