#ifndef LPF_H
#define LPF_H
#include "main.h"


typedef struct
{
float LPF_K;//�˲�ϵ��
int value_last;//��һ�ε�ֵ
int value_new;//��һ�ε�ֵ	
	
int value_RESUIT;//���ֵ	

}LPF_t;
extern LPF_t SPEED_L;
extern LPF_t SPEED_R;
extern LPF_t milemeter_A;


extern float LPF_a;//
extern float LPF_b;//
//��̥��̼��˲���

int LPF_V1(int value_wait_lpf,float LPF_k);
int LPF_V2(LPF_t *LPF_VOID,int value_new_void);


#endif
