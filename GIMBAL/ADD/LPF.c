#include "LPF.h"


float LPF_a=0.8f;//0.85运动会丝滑,抖动会减少,但是没那么精准.灵敏
//轮胎速度滤波用

float LPF_b=0.8f;//

//轮胎里程计滤波用
int LPF_V1(int value_wait_lpf,float LPF_k)
{
static int value_new;
static int value_new_last;
	
value_new=(1-LPF_k)*value_new_last+LPF_k*value_wait_lpf;

value_new_last=value_new;
return value_new;

}

LPF_t SPEED_L;
LPF_t SPEED_R;
LPF_t milemeter_A;
int LPF_V2(LPF_t *LPF_VOID,int value_new_void)
{
	LPF_VOID->value_new=value_new_void;
	
LPF_VOID->value_RESUIT=(1-LPF_VOID->LPF_K)*LPF_VOID->value_last+LPF_VOID->LPF_K*LPF_VOID->value_new;
	

LPF_VOID->value_last=LPF_VOID->value_RESUIT;
	
return LPF_VOID->value_RESUIT;

}








