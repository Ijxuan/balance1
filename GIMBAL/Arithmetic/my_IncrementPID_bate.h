#ifndef PID_Increment_h
#define PID_Increment_h

#include "main.h"

//#include "User_math.h"
#include "Math.h"

#define I_PID_HOOK_FUN {\
  I_PID_Parameter_Init,\
  I_PID_Regulation,\
  I_PID_Parameter_Clear,\
}
extern uint16_t beta;
typedef struct
{
  float Kp; //����ϵ��
  float Ki; //����ϵ��
  float Kd; //΢��ϵ��

  float Target;  //Ŀ��ֵ
  float Measure; //����ֵ

  float Error;     //ƫ��ֵ
  float Epsilon;   //ƫ������ֵ
  float max_error; //ƫ������ֵ
  float min_error; //ƫ�����Сֵ

  float Proportion;   //����ֵ
  float Integral;     //����ֵ
  float Differential; //΢��ֵ

  //����ȫ΢��
  float alpha;         //����ȫ΢��ϵ��
  float D_Output;      //΢�����
  float D_Last_Output; //��һ�̵�΢�����

  float Max_antiwindup;       //�����ֱ��͵�������ֵ
  float Min_antiwindup;       //�����ֱ��͵������Сֵ

  float Increment_Output;     //�������
  float result;     //PID����ṹ
  float Max_result; //result���ֵ
  float Min_result; //result��Сֵ

  float LastError; //ǰһ��ƫ��
  float PreError;  //ǰ����ƫ��

} I_PID_t;
extern I_PID_t CHASSIS_I_PID;
extern I_PID_t SHOOT_L_I_PID;
extern I_PID_t SHOOT_R_I_PID;
extern I_PID_t Driver_I_PID;
typedef struct 
{
  void (*I_PID_Parameter_Init)(I_PID_t *I_PID, float Kp, float Ki, float Kd,
                          float epsilon, float max_error, float min_error,
                          float alpha,
                          float Max_antiwindup, float Min_antiwindup,
                          float Max_result, float Min_result);
  float (*I_PID_Regulation)(I_PID_t *I_PID, float target, float measure);
  void (*I_PID_Parameter_Clear)(I_PID_t *I_PID);
}
I_PID_FUN_t;

extern I_PID_FUN_t I_PID_FUN;


void I_PID_Parameter_Init(I_PID_t *I_PID, float Kp, float Ki, float Kd,
                          float epsilon, float max_error, float min_error,
                          float alpha,
                          float Max_antiwindup, float Min_antiwindup,
                          float Max_result, float Min_result);

float I_PID_Regulation(I_PID_t *I_PID, float target, float measure);
													
													
void I_PID_Parameter_Clear(I_PID_t *I_PID);

#endif
