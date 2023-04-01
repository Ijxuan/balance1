#ifndef mit_math_H
#define mit_math_H

#include "MIT.h"
#include "math.h"
#include "User_math.h"

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

#endif
