#include "LPF.h"


float LPF_a=0.85f;//0.85运动会丝滑,抖动会减少,但是没那么精准.灵敏
	
int LPF_V1(int value_wait_lpf)
{
static int value_new;
static int value_new_last;
	
value_new=(1-LPF_a)*value_new_last+LPF_a*value_wait_lpf;

value_new_last=value_new;
return value_new;

}








