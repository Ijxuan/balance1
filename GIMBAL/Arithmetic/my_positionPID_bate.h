#ifndef PID_H
#define PID_H

#include "main.h"
//#include "user_CAN.h"
//#include "PID_Position.h"
//#include "PID_Increment.h"
//#include "BSP_CAN.h"
#include "INS_task.h"

#define abs(x) ((x)>0?(x):-(x))
#define PID_MOTOR 0 //�Ƿ��������PID  0�ر�
#define PID_YAW_IMU 1 //�Ƿ��������ǵ�PID
#define VISION_PID_YAW_IMU 1 //�Ƿ����Ӿ������ǵ�PID

#define PID_PITCH_MOTOR 1 //�Ƿ��������PITCH��PID  0�ر�
#define PID_PITCH_IMU 1 //�Ƿ��������ǵ�PITCH��PID  0�ر�
#define PID_CHASSIS_MOTOR 0//�Ƿ�������ĵ���PID  0�ر�
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

  float result;     //PID����ṹ
  float Max_result; //result���ֵ
  float Min_result; //result��Сֵ

  float LastError; //ǰһ��ƫ��
  float PreError;  //ǰ����ƫ��
  
  float P_Output;      //�������
  float I_Output;      //�������


} P_PID_t;
float P_PID_bate(P_PID_t *P_PID, float target, float measure);
void P_PID_Parameter_Init(P_PID_t *P_PID, float Kp, float Ki, float Kd,
                          float epsilon,
//float max_error, float min_error,
//                          float alpha,
                          float Max_antiwindup, float Min_antiwindup,
                          float Max_result, float Min_result);
						  
void P_PID_Parameter_Clear(P_PID_t *P_PID);


extern int PID_YES;
#if PID_MOTOR
extern P_PID_t Yaw_Angle_pid;
extern P_PID_t Yaw_Speed_pid;
#endif
						  
#if PID_YAW_IMU
extern P_PID_t Yaw_IMU_Angle_pid;
extern P_PID_t Yaw_IMU_Speed_pid;
#endif
		
#if VISION_PID_YAW_IMU
extern P_PID_t VISION_Yaw_IMU_Angle_pid;
extern P_PID_t VISION_Yaw_IMU_Speed_pid;
#endif
						  
#if PID_PITCH_MOTOR
extern P_PID_t PITCH_Angle_pid;
extern P_PID_t PITCH_Speed_pid;
#endif
						  
#if PID_PITCH_IMU
extern P_PID_t PITCH_IMU_Angle_pid;
extern P_PID_t PITCH_IMU_Speed_pid;
#endif

#if PID_CHASSIS_MOTOR
extern P_PID_t CHASSIS_MOTOR_ANGLE_pid;
extern P_PID_t CHASSIS_MOTOR_SPEED_pid;
#endif


extern P_PID_t Driver_ANGLE_pid;
extern P_PID_t Driver_SPEED_pid;

extern P_PID_t TIRE_R_ANGLE_pid;//轮胎PID
extern P_PID_t TIRE_R_SPEED_pid;

extern P_PID_t TIRE_L_ANGLE_pid;//轮胎PID
extern P_PID_t TIRE_L_SPEED_pid;	
extern P_PID_t BALANCE_P;	
extern P_PID_t BALANCE_I;

extern P_PID_t SPEED_P;//速度环P


extern P_PID_t change_direction_speed;//转向环->速度环
extern P_PID_t change_direction_angle;//转向环->角度环

#endif

