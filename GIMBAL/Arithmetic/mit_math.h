#ifndef mit_math_H
#define mit_math_H

#include "MIT.h"
#include "math.h"
#include "User_math.h"
#include "mit_math.h"

#define use_MIT_Accurately 1

#define use_MIT_change_focus 0

void mit_math_temp(float fai1, float fai2, float *X_now, float *Y_now); /*ƽ������������*/
void mit_math_temp_2(float Cx, float Cy);                               ///*ƽ�����������*/
void get_tg_angle_by_WLG_IS(void);                                      ///*ͨ��ƽ�������������Ŀ��Ƕ�*/
void Accurately_contrul_text(void);                                     ///*ͨ��ƽ�������������Ŀ��ǶȾ�ȷ���Ʋ���*/
void MIT_keep_BALENCE(void);                                            // �ؽڵ��ʱ�̱���ƽ��
void update_gyro(void);
void gyro_test(void);
void engine_body_height_control(void);
void MIT_c_get_xy_speed(float * x_speed_R,float * y_speed_R , float * x_speed_L,float * y_speed_L
	,float * swing_link_speed_L,float * swing_link_speed_R);
void MIT_orque_TG(void);/*MIT目标力矩计算函数*/

extern float angle_fai_1;    // ����ϵԭ�㴦����ˮƽ��ļн�(���)
extern float angle_fai_2;    //(20,0)������ˮƽ��ļн�(���)
extern float angle_fai_1_JD; // ����ϵԭ�㴦����ˮƽ��ļн�(���) �Ƕ���
extern float angle_fai_2_JD; //(20,0)������ˮƽ��ļн�(���) �Ƕ���

extern float MIT_A_tg_angle_for_IS; // ����ϵԭ�㴦�ؽڵ��Ŀ��Ƕ�
extern float MIT_B_tg_angle_for_IS; //(20,0)���ؽڵ��Ŀ��Ƕ�
extern float A0_TEMP;
extern float B0_TEMP;
extern float C0_TEMP;
extern float TEMP_SQRT;
extern int sqrt_allow; // �ܲ�����ƽ����

extern float keep_BALENCE_by_MIT_RT;

extern Ramp_Struct MIT_BALENCE_start;
extern Ramp_Struct MIT_BALENCE_GO_TO_TG;

extern int if_use_Ramp_Function; // �Ƿ�ʹ��б�º���
extern float angle_qhq;          // ǰ����б�ĽǶ�
extern float angle_qhq_pi;       // ǰ����б�ĽǶ� ������
extern float C_x_now;
extern float C_y_now;

extern float angle_fai_1_zhen; // ����ϵԭ�㴦����ˮƽ��ļн�(���) ����
extern float angle_fai_2_zhen; //(20,0)������ˮƽ��ļн�(���) ����
extern float angle_fai_3_zhen; // �ڶ�����ϥ�Ǵ��н� ����
extern float A0_TEMP_zhen;     // ����
extern float B0_TEMP_zhen;     // ����
extern float C0_TEMP_zhen;     // ����

extern float T1_Q ;					 // 坐标系原点处腿与水平面的夹角(顿角)
extern float T2 ;					 // (20,0) E 点处腿与水平面的夹角(锐角)


extern int cumulate_time_ms;        // �ۻ�ʱ��
extern float cumulate_angle_change; // �ۻ��Ƕȱ仯
extern float pitch_speed_new;       // �������pitch����ٶ�

extern float cumulate_angle_change_JD; // �ۻ��Ƕȱ仯 �Ƕ���
extern float pitch_speed_new_JD;       // �������pitch����ٶ� _�Ƕ���

extern int speed_to_angle_time_ms; // ���ֽ��ٶ�-ʱ��
extern float change_angle_1ms;     // 1ms�Ļ���
//	float change_angle_total_speed=0;//�����ۼӵ��м����
extern float change_angle_total_speed_end; // �ۼ����ֵ

extern float change_angle_total_angle; // ʵ�ʽǶȱ仯��-���ڶԱ�
extern Ramp_Struct liftoff_SE;         // ��ظ߶�б��
extern float height_text;

extern float F_y_R;//右边腿的支撑力

extern float swing_link_length_R;//右腿摆长
extern float swing_link_length_L;//左腿摆长
extern  int liftoff_mode_T ;
extern float engine_body_height_tg ;//目标离地高度

#endif
