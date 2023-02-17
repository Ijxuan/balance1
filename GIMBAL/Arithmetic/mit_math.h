#ifndef mit_math_H
#define mit_math_H

#include "MIT.h"
#include "math.h"


void mit_math_temp(void);/*平面五连杆正解*/
void mit_math_temp_2(float Cx,float Cy);///*平面五连杆逆解*/

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

#endif




