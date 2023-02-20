#ifndef LPF_H
#define LPF_H
#include "main.h"


typedef struct
{
float LPF_K;//滤波系数
int value_last;//上一次的值
int value_new;//这一次的值	
	
int value_RESUIT;//输出值	

}LPF_t;
extern LPF_t SPEED_L;
extern LPF_t SPEED_R;
extern LPF_t milemeter_A;
extern LPF_t SPEED_MIT;
extern LPF_t SPEED_MIT_A;
extern LPF_t SPEED_MIT_B;
extern LPF_t SPEED_MIT_C;
extern LPF_t SPEED_MIT_D;

extern float LPF_a;//
extern float LPF_b;//
//轮胎里程计滤波用
extern LPF_t SPEED_L_FOR_MIT;
extern LPF_t SPEED_R_FOR_MIT;
int LPF_V1(int value_wait_lpf,float LPF_k);
int LPF_V2(LPF_t *LPF_VOID,int value_new_void);


#endif
