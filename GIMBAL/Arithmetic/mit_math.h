#ifndef mit_math_H
#define mit_math_H

#include "MIT.h"
#include "math.h"
#include "User_math.h"

#define use_MIT_Accurately 1

#define use_MIT_change_focus 0

void mit_math_temp(void);/*ƽ������������*/
void mit_math_temp_2(float Cx,float Cy);///*ƽ�����������*/
void get_tg_angle_by_WLG_IS(void);///*ͨ��ƽ�������������Ŀ��Ƕ�*/
void Accurately_contrul_text(void);///*ͨ��ƽ�������������Ŀ��ǶȾ�ȷ���Ʋ���*/
void MIT_keep_BALENCE(void);//�ؽڵ��ʱ�̱���ƽ��

extern float angle_fai_1;//����ϵԭ�㴦����ˮƽ��ļн�(���)
extern float angle_fai_2;//(20,0)������ˮƽ��ļн�(���)
extern float angle_fai_1_JD;//����ϵԭ�㴦����ˮƽ��ļн�(���) �Ƕ���
extern float angle_fai_2_JD;//(20,0)������ˮƽ��ļн�(���) �Ƕ���

extern float MIT_A_tg_angle_for_IS;//����ϵԭ�㴦�ؽڵ��Ŀ��Ƕ�
extern float MIT_B_tg_angle_for_IS;//(20,0)���ؽڵ��Ŀ��Ƕ�
extern float A0_TEMP;
extern float B0_TEMP;
extern float C0_TEMP;
extern float TEMP_SQRT;
extern int sqrt_allow;//�ܲ�����ƽ����

extern float keep_BALENCE_by_MIT_RT;

extern Ramp_Struct MIT_BALENCE_start;

#endif




