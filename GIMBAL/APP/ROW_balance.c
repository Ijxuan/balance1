#include "ROW_balance.h"
#include "my_positionPID_bate.h"

/*
主要功能：
保持机体row轴水平

测量值：陀螺仪row轴角度  左腿实际腿长 右腿实际腿长

输出值：左/右腿目标腿长

逻辑：根据实际腿长：较短的那边等于目标高度  实时调整另一边腿长保持row轴水平

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
void row_control()
{

//if(R_C_Y_NOW<L_C_Y_NOW)
//{
//R_C_Y_NOW=14;//右腿是短的那边，右腿强制等于目标高度
//}
//else if(R_C_Y_NOW>L_C_Y_NOW)
//{
//L_C_Y_NOW=14;//左腿是短的那边，左腿强制等于目标高度
//}
engine_body_height_R=14;
leg_L_change_row=P_PID_bate(&keep_ROW_BALENCE,0,DJIC_IMU.Row);
engine_body_height_L=14+leg_L_change_row;
	
}







