#ifndef mit_math_H
#define mit_math_H

#include "MIT.h"
#include "math.h"
#include "User_math.h"

#define use_MIT_Accurately 1

#define use_MIT_change_focus 0

void mit_math_temp(float fai1,float fai2,float *X_now,float *Y_now);/*平面五连杆正解*/
void mit_math_temp_2(float Cx,float Cy);///*平面五连杆逆解*/
void get_tg_angle_by_WLG_IS(void);///*通过平面五连杆逆解获得目标角度*/
void Accurately_contrul_text(void);///*通过平面五连杆逆解获得目标角度精确控制测试*/
void MIT_keep_BALENCE(void);//关节电机时刻保持平衡
void update_gyro(void);
void gyro_test(void);
void engine_body_height_control(void);

extern float angle_fai_1;//坐标系原点处腿与水平面的夹角(锐角)
extern float angle_fai_2;//(20,0)处腿与水平面的夹角(锐角)
extern float angle_fai_1_JD;//坐标系原点处腿与水平面的夹角(锐角) 角度制
extern float angle_fai_2_JD;//(20,0)处腿与水平面的夹角(锐角) 角度制

extern float MIT_A_tg_angle_for_IS;//坐标系原点处关节电机目标角度
extern float MIT_B_tg_angle_for_IS;//(20,0)处关节电机目标角度
extern float A0_TEMP;
extern float B0_TEMP;
extern float C0_TEMP;
extern float TEMP_SQRT;
extern int sqrt_allow;//能不能求平方根

extern float keep_BALENCE_by_MIT_RT;

extern Ramp_Struct MIT_BALENCE_start;
extern Ramp_Struct MIT_BALENCE_GO_TO_TG;

extern int if_use_Ramp_Function;//是否使用斜坡函数
extern float angle_qhq;//前后倾斜的角度
extern float angle_qhq_pi;//前后倾斜的角度 弧度制
extern float C_x_now;
extern float C_y_now;

extern float angle_fai_1_zhen;//坐标系原点处腿与水平面的夹角(锐角) 正解
extern float angle_fai_2_zhen;//(20,0)处腿与水平面的夹角(锐角) 正解
extern float angle_fai_3_zhen;//第二象限膝盖处夹角 正解
extern float A0_TEMP_zhen;//正解
extern float B0_TEMP_zhen;//正解
extern float C0_TEMP_zhen;//正解

extern int cumulate_time_ms;//累积时间
extern float cumulate_angle_change;//累积角度变化
extern float pitch_speed_new;//计算出的pitch轴角速度

extern float cumulate_angle_change_JD;//累积角度变化 角度制
extern float pitch_speed_new_JD;//计算出的pitch轴角速度 _角度制

extern  int speed_to_angle_time_ms;//积分角速度-时间
extern	float change_angle_1ms;//1ms的积分
//	float change_angle_total_speed=0;//用于累加的中间变量
extern	float change_angle_total_speed_end;//累加完成值

extern	float change_angle_total_angle;//实际角度变化量-用于对比
extern	Ramp_Struct liftoff_SE;//离地高度斜坡
extern float height_text;

#endif




