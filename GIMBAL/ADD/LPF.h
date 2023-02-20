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
extern LPF_t SPEED_MIT;
extern LPF_t SPEED_MIT_A;
extern LPF_t SPEED_MIT_B;
extern LPF_t SPEED_MIT_C;
extern LPF_t SPEED_MIT_D;

extern float LPF_a;//
extern float LPF_b;//
//��̥��̼��˲���
extern LPF_t SPEED_L_FOR_MIT;
extern LPF_t SPEED_R_FOR_MIT;
int LPF_V1(int value_wait_lpf,float LPF_k);
int LPF_V2(LPF_t *LPF_VOID,int value_new_void);


#endif
