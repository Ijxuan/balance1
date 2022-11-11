#include "LPF.h"


float LPF_a=1.0f;
	
int LPF_V1(int value_wait_lpf)
{
static int value_new;
static int value_new_last;
	
value_new=(1-LPF_a)*value_new_last+LPF_a*value_wait_lpf;

value_new_last=value_new;
return value_new;

}








