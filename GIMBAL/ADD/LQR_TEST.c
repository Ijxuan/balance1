#include "LQR_TEST.h"
#include "DR16_RECIVE.h"
#include "M3508.h"
#include "INS_task.h"
#include "user_lib.h"
#include "MY_balance_CONTROL.h"
#include "my_positionPID_bate.h"
#include "mit_math.h"

static void chassis_balance_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_remote_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

float  Nm_L_test;
float  Nm_R_test;

int send_to_L_test;
int send_to_R_test;

float LQR_TARGET_position=0;

//底盘运动数据
chassis_move_t chassis_move;

void LQR_TEST_CON()
{

        //chassis data update
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
	
        //set chassis control set-point 
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);

	        //chassis control LQR calculate
        //底盘控制LQR计算
        chassis_control_loop(&chassis_move);
}

/**
  * @brief          设置控制目标量.根据不同底盘控制模式，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     angle_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
  * @retval         none
  */
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

//    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
//    {
//      return;
//    }

//    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
//    {
//			chassis_down_control(vx_set, angle_set, chassis_move_rc_to_vector);
//    }
//	*vx_set=DR16.rc.ch1;

 if (DR16.rc.s_right == 3&& DR16.rc.s_left==3)
    {
		chassis_remote_control(vx_set, angle_set, chassis_move_rc_to_vector);//遥控器控制模式

//			chassis_balance_control(vx_set, angle_set, chassis_move_rc_to_vector);//平衡模式
    }
 if (DR16.rc.s_right == 2)
    {
//			chassis_down_control(vx_set, angle_set, chassis_move_rc_to_vector);
    }	
 if (DR16.rc.s_right == 1)
    {
//chassis_remote_control(vx_set, angle_set, chassis_move_rc_to_vector);//遥控器控制模式
    }	
	
	
//    else if (chassis_behaviour_mode == CHASSIS_REMOTE)
//    {
//			chassis_remote_control(vx_set, angle_set, chassis_move_rc_to_vector);
//    }
}


/**
  * @brief          底盘遥控的行为状态机下，目标速度是摇杆3通道的映射，目标转角是摇杆2通道的映射
  * @param[in]      vx_set 前进的速度,正值表示前进速度，负值表示后退速度
  * @param[in]      angle_set 目标转角，范围-PI到PI
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_remote_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
//    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
//    {
//        return;  //
//    }

    chassis_rc_to_control_vector(vx_set, chassis_move_rc_to_vector);
		
    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set
	- CHASSIS_ANGLE_Z_RC_SEN * DR16.rc.ch0);
}









/**
  * @brief          accroding to the channel value of remote control, calculate chassis vertical speed set-point
  *                 
  * @param[out]     vx_set: vertical speed set-point
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          根据遥控器通道值，计算纵向速度设定值
  *                 
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
      int16_t vx_channel ;
    fp32 vx_set_channel ;
void chassis_rc_to_control_vector(fp32 *vx_set, chassis_move_t *chassis_move_rc_to_vector)
{
//    if (chassis_move_rc_to_vector == NULL || vx_set == NULL )
//    {
//        return;
//    }
    

		
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
//    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
		
	vx_channel=DR16.rc.ch1*-1;
	
    //将遥杆参数转换为运动参数
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN+TARGET_SPEED_POSITION;
		
    //keyboard set speed set-point
    //键盘控制
//    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
//    {
//        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
//    }
//    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
//    {
//        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
//    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //一阶低通滤波代替斜波作为底盘速度输入
//    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    
		//stop command, need not slow change, set zero derectly
    //停止信号，不需要缓慢加速，直接减速到零
//    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
//    {
//        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
//    }

    *vx_set = vx_set_channel;
}


/**
  * @brief          底盘DOWN掉的行为状态机下，目标速度和目标转角均为0
  * @param[in]      vx_set 前进的速度,正值表示前进速度，负值表示后退速度
  * @param[in]      angle_set 目标转角
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_down_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
      return;
    }
    *vx_set = 0.0f;
    *angle_set = 0.0f;
}

/**
  * @brief          底盘平衡的行为状态机下，目标速度和目标转角均为0
  * @param[in]      vx_set 前进的速度,正值表示前进速度，负值表示后退速度
  * @param[in]      angle_set 目标转角
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_balance_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
//    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
//    {
//      return;
//    }
    *vx_set = 0.0f;
    *angle_set = 0.0f;
}
float K3_OUT=0;
float K4_OUT=0;
float K2_OUT=0;
float TARGET_SPEED_POSITION;
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	    fp32 max_torque = 0.0f, torque_rate = 0.0f;
    fp32 temp = 0.0f;
    uint8_t i = 0;
		/*驱动轮输出力矩=SUM[LQR增益系数*(状态变量目标值-状态变量反馈值)]*/
		/*注意左右轮输出力矩的正负号，以及各状态变量反馈值的正负号*/
	
	if(R_Y<30)
	{
	
	  //左轮输出力矩
 Nm_R_test = - (
	LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) 
	+
	LQR_K3*chassis_move_control_loop->chassis_pitch
	- 
	LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed)
	);
	
		  //右轮输出力矩
Nm_L_test  =   (
	LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) 
+
	LQR_K3*chassis_move_control_loop->chassis_pitch  
	- 
	LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed) 
	);
}
else 	if(R_Y>=30)
{
	  //左轮输出力矩
 Nm_R_test = - (
	M_LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) 
	+
	M_LQR_K3*chassis_move_control_loop->chassis_pitch
	- 
	M_LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed)
//	+ 
//	LQR_K15*chassis_move_control_loop->delta_angle 
//	- 
//	LQR_K16*chassis_move_control_loop->chassis_yaw_speed
	);
//K2_OUT=	LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set);
//K3_OUT=	LQR_K3*chassis_move_control_loop->chassis_pitch;
//K4_OUT=LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed) ;	
	
	
		  //右轮输出力矩
Nm_L_test  =   (
	M_LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) 
+
	M_LQR_K3*chassis_move_control_loop->chassis_pitch  
	- 
	M_LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed) 
//	+ 
//	LQR_K25*chassis_move_control_loop->delta_angle 
//	- 
//	LQR_K26*chassis_move_control_loop->chassis_yaw_speed
	);

}


    //计算轮子最大转矩，并限制其最大转矩
			temp = fabs(Nm_R_test);
			if (max_torque < temp)
			{
				max_torque = temp;
			}
			
			temp = fabs(Nm_L_test);
			if (max_torque < temp)
			{
				max_torque = temp;
			}			


    if (max_torque > MAX_WHEEL_TORQUE)
    {
			torque_rate = MAX_WHEEL_TORQUE / max_torque;

				send_to_L_test *= torque_rate;
				send_to_R_test *= torque_rate;

    }   

	//赋值电流值

send_to_L_test = (int16_t)(Nm_L_test / CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN);
    
send_to_R_test = (int16_t)(Nm_R_test / CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN);
	

	
if(DR16.rc.s_left==3&&DR16.rc.s_right==3)
{
send_to_tire_L=send_to_L_test;//将LQR算出来的值赋给发送变量
send_to_tire_R=send_to_R_test;

	
	TARGET_SPEED_POSITION=P_PID_bate(&LQR_SPEED_BY_POSITION,LQR_TARGET_position,milemeter_test.total_mile_truly_use)/1000.0f;

	if(DR16.rc.ch1!=0)//前进时更新目标位置
{
LQR_TARGET_position=milemeter_test.total_mile_truly_use;
TARGET_SPEED_POSITION=0;
}
	if(DR16.rc.ch0!=0)//转向时更新目标位置
{
LQR_TARGET_position=milemeter_test.total_mile_truly_use;
TARGET_SPEED_POSITION=0;
}
	if(DW_FOR_ZX!=0)//小陀螺时更新目标位置
{
LQR_TARGET_position=milemeter_test.total_mile_truly_use;
TARGET_SPEED_POSITION=0;
}
	if(DR16.rc.ch0!=0)
	{
	TARGET_angle_YAW=DJIC_IMU.total_yaw+DR16.rc.ch0/15.0;
	}		
	TARGET_angle_speed_YAW=P_PID_bate(&change_direction_angle,TARGET_angle_YAW,DJIC_IMU.total_yaw);
	if(DW_FOR_ZX!=0)//拨轮控制,控制速度
	{
	TARGET_angle_speed_YAW=DW_FOR_ZX;
	TARGET_angle_YAW	=DJIC_IMU.total_yaw;
	}
send_to_tire_L+=P_PID_bate(&change_direction_speed,TARGET_angle_speed_YAW,DJIC_IMU.Gyro_z);
send_to_tire_R+=P_PID_bate(&change_direction_speed,TARGET_angle_speed_YAW,DJIC_IMU.Gyro_z);
	
}
else
{
LQR_TARGET_position=milemeter_test.total_mile_truly_use;
}



}

















/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
//void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
//{
//    first_order_filter_type->frame_period = frame_period;
//    first_order_filter_type->num[0] = num[0];
//    first_order_filter_type->input = 0.0f;
//    first_order_filter_type->out = 0.0f;
//}


/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
//void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
//{
//    first_order_filter_type->input = input;
//    first_order_filter_type->out =
//        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
//}


//循环限幅函数
//fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
//{
//    if (maxValue < minValue)
//    {
//        return Input;
//    }

//    if (Input > maxValue)
//    {
//        fp32 len = maxValue - minValue;
//        while (Input > maxValue)
//        {
//            Input -= len;
//        }
//    }
//    else if (Input < minValue)
//    {
//        fp32 len = maxValue - minValue;
//        while (Input < minValue)
//        {
//            Input += len;
//        }
//    }
//    return Input;
//}

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
//    if (chassis_move_update == NULL)
//    {
//        return;
//    }

    uint8_t i = 0;
    for (i = 0; i < 2; i++)
    {
			//update motor speed
			//更新电机轮轴位移速度
			chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * M3508s[2+i].realSpeed;
			//i=0是右轮   i=1是左轮
			//update motor angular velocity
			//更新电机输出轴角速度
			chassis_move_update->motor_chassis[i].omg = CHASSIS_MOTOR_RPM_TO_OMG_SEN * M3508s[2+i].realSpeed;
			
			//update motor torque
			//更新电机转矩
			chassis_move_update->motor_chassis[i].torque = CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN * M3508s[2+i].realCurrent;
    }

    //calculate chassis euler angle
    //计算底盘姿态角度
    chassis_move_update->chassis_yaw = 
	rad_format((float)INS_angle[0]);
//	if(DR16.rc.ch1==0)
    chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2]-angle_qhq_pi);
//	else
//	{
//    chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2]);
//	}
//	    chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2]);

    chassis_move_update->chassis_roll = (float)INS_angle[1];
		
		//calculate chassis euler angular velocity
		//计算底盘姿态角速度
		chassis_move_update->chassis_yaw_speed = INS_gyro[2];
		
	chassis_move_update->chassis_pitch_speed =  INS_gyro[0]-pitch_speed_new;
		
	chassis_move_update->chassis_roll_speed = INS_gyro[1];	  
		//calculate chassis velocity and synthesis angular velocity
		//计算底盘速度和合成轮角速度
		chassis_move_update->vx = ((chassis_move_update->motor_chassis[0].speed) - (chassis_move_update->motor_chassis[1].speed))/2 ;
	  chassis_move_update->omg = ((chassis_move_update->motor_chassis[0].omg) - (chassis_move_update->motor_chassis[1].omg))/2 ;
}


/**
  * @brief          set chassis control target-point, movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制目标值, 运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
      return;
    }

    fp32 vx_set = 0.0f, angle_set = 0.0f;
		
    //get movement control target-points, 获取运动控制目标值
    chassis_behaviour_control_set(&vx_set, &angle_set, chassis_move_control);
    chassis_move_control->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_control->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
//    if (chassis_move_control->chassis_mode == CHASSIS_REMOTE_MODE)
//    {
//			chassis_move_control->delta_angle = 0.0f;
//			chassis_move_control->chassis_yaw_set = rad_format(angle_set);
//			chassis_move_control->delta_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
//			chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
//    }
//		else
			if (DR16.rc.s_right == 3&& DR16.rc.s_left==3)
    {
			chassis_move_control->chassis_yaw_set = rad_format(angle_set);
			chassis_move_control->delta_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
			chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
	else
	{
	chassis_move_control->chassis_yaw_set = chassis_move_control->chassis_yaw;
		angle_set=rad_format((float)INS_angle[0]);
	}
//		else if (chassis_move_control->chassis_mode == CHASSIS_DOWN_MODE)
//		{
//			chassis_move_control->chassis_yaw_set = rad_format(angle_set);
//			chassis_move_control->delta_angle = chassis_move_control->chassis_yaw_set;
//      chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
//    }
}


////限幅函数
//fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
//{
//    if (Value < minValue)
//        return minValue;
//    else if (Value > maxValue)
//        return maxValue;
//    else
//        return Value;
//}

