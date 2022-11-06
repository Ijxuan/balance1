#ifndef PID_H
#define PID_H

#include "main.h"
//#include "user_CAN.h"
//#include "PID_Position.h"
//#include "PID_Increment.h"
//#include "BSP_CAN.h"
#include "INS_task.h"

#define abs(x) ((x)>0?(x):-(x))
#define PID_MOTOR 0 //是否开启电机的PID  0关闭
#define PID_YAW_IMU 1 //是否开启陀螺仪的PID
#define VISION_PID_YAW_IMU 1 //是否开启视觉陀螺仪的PID

#define PID_PITCH_MOTOR 1 //是否开启电机的PITCH轴PID  0关闭
#define PID_PITCH_IMU 1 //是否开启陀螺仪的PITCH轴PID  0关闭
#define PID_CHASSIS_MOTOR 0//是否开启电机的底盘PID  0关闭
typedef struct
{
  float Kp; //比例系数
  float Ki; //积分系数
  float Kd; //微分系数

  float Target;  //目标值
  float Measure; //测量值

  float Error;     //偏差值
  float Epsilon;   //偏差检测阈值
  float max_error; //偏差的最大值
  float min_error; //偏差的最小值

  float Proportion;   //比例值
  float Integral;     //积分值
  float Differential; //微分值

  //不完全微分
  float alpha;         //不完全微分系数
  float D_Output;      //微分输出
  float D_Last_Output; //上一刻的微分输出

  float Max_antiwindup;       //抗积分饱和的输出最大值
  float Min_antiwindup;       //抗积分饱和的输出最小值

  float result;     //PID计算结构
  float Max_result; //result最大值
  float Min_result; //result最小值

  float LastError; //前一拍偏差
  float PreError;  //前两拍偏差
  
  float P_Output;      //比例输出
  float I_Output;      //积分输出


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

						  
#endif

