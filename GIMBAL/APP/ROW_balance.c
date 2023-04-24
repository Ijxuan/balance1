#include "ROW_balance.h"
#include "my_positionPID_bate.h"
#include "math.h"
#include "mit_math.h"

/*
主要功能：
保持机体row轴水平

测量值：陀螺仪row轴角度  左腿实际腿长 右腿实际腿长

输出值：左/右腿目标腿长

逻辑：根据实际腿长：较短的那边等于目标高度  实时调整另一边腿长保持row轴水平
新逻辑：两腿长度的平均值作为高度，ROW轴保持平衡的值直接叠加到目标y上

至此，目标Y将有三部分组成：目标机体高度（固定--斜坡增减）+ROW轴保持平衡的值（PID计算-时刻刷新）+跳跃伸长量（突变）

右腿抬高：DJIC_IMU.Row为正值
左腿伸长：DJIC_IMU.Row为负值
*/

/*
R_C_Y_NOW   右腿实际腿长
L_C_Y_NOW  左腿实际腿长

engine_body_height_R 目标
engine_body_height_L 目标

DJIC_IMU.Row
*/
float leg_L_change_row;
float body_width=52.3;//车体宽度
float real_engine_body_height;//真实机体高度

void row_control()
{
leg_L_change_row=P_PID_bate(&keep_ROW_BALENCE,0,DJIC_IMU.Row);

//if(R_C_Y_NOW<L_C_Y_NOW)
//{
//engine_body_height_R=engine_body_height_tg;//右腿是短的那边，右腿强制等于目标高度
//engine_body_height_L=engine_body_height_tg+leg_L_change_row;
//	
//}
//else if(R_C_Y_NOW>L_C_Y_NOW)
//{
//engine_body_height_L=engine_body_height_tg;//左腿是短的那边，左腿强制等于目标高度
//engine_body_height_R=engine_body_height_tg-leg_L_change_row;

//}
engine_body_height_L=leg_L_change_row/2.0f;
engine_body_height_R=-leg_L_change_row/2.0f;;

//engine_body_height_R=14-leg_L_change_row;
//	engine_body_height_L=14;
//	leg_L_change_row=L_C_Y_NOW+sin(DJIC_IMU.Row)*body_width;
//	engine_body_height_R=14;

//	engine_body_height_L=14+leg_L_change_row;

}

//void get_real_engine_body_height()
//{

//}




