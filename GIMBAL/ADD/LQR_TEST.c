#include "LQR_TEST.h"
#include "DR16_RECIVE.h"
#include "M3508.h"
#include "INS_task.h"
#include "user_lib.h"
#include "MY_balance_CONTROL.h"
#include "my_positionPID_bate.h"
#include "mit_math.h"
#include "math.h"
#include "keyBoard_to_vjoy.h"
#include "CHASSIS_follow.h"

static void chassis_balance_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_remote_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

float Nm_L_test;
float Nm_R_test;

int send_to_L_test;
int send_to_R_test;

float LQR_TARGET_position = 0;

// 底盘运动数据
chassis_move_t chassis_move;
float LQRweiyi_text = 0;
float LQRweiyi_PO_TG = 0;	 // lqr位移目标
float LQRweiyi_SPEED_TG = 0; // LQR速度目标

double LQR_K1_REAL_TIME;
double LQR_K2_REAL_TIME;
double LQR_K3_REAL_TIME;//多项式拟合
double LQR_K4_REAL_TIME;//多项式拟合
double LQR_K3_REAL_TIME_xx;//线性拟合
double LQR_K4_REAL_TIME_xx;//线性拟合

void LQR_TEST_CON()
{

	// chassis data update
	// 底盘数据更新
	chassis_feedback_update(&chassis_move);

	// set chassis control set-point
	// 底盘控制量设置
	chassis_set_contorl(&chassis_move);

	//LQR参数刷新
	LQR_parameter();

	// chassis control LQR calculate
	// 底盘控制LQR计算
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

	if (DR16.rc.s_right == 3 && DR16.rc.s_left == 3)
	{
		chassis_remote_control(vx_set, angle_set, chassis_move_rc_to_vector); // 遥控器控制模式

		//			chassis_balance_control(vx_set, angle_set, chassis_move_rc_to_vector);//平衡模式
	}
	if (DR16.rc.s_right == 2)
	{
		//			chassis_down_control(vx_set, angle_set, chassis_move_rc_to_vector);
	}
	if (DR16.rc.s_right == 3 && DR16.rc.s_left == 1)
	{
		chassis_remote_control(vx_set, angle_set, chassis_move_rc_to_vector); // 遥控器控制模式

		//			chassis_balance_control(vx_set, angle_set, chassis_move_rc_to_vector);//平衡模式
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

	*angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * DR16.rc.ch0);
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
int16_t vx_channel;
fp32 vx_set_channel;
void chassis_rc_to_control_vector(fp32 *vx_set, chassis_move_t *chassis_move_rc_to_vector)
{
	//    if (chassis_move_rc_to_vector == NULL || vx_set == NULL )
	//    {
	//        return;
	//    }

	// deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
	// 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
	//    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);

	vx_channel = DR16.rc.ch3 * -1;
	vx_channel = 0;

	get_speed_by_position_V1(); // 计算TARGET_SPEED_POSITION
	TARGET_SPEED_POSITION = 0;	// LQR有位置环，不劳速度费心了
	get_speed_by_position_V2();
	// 将遥杆参数转换为运动参数
	vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN + TARGET_SPEED_POSITION_V2;

	// keyboard set speed set-point
	// 键盘控制
	//    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
	//    {
	//        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
	//    }
	//    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
	//    {
	//        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
	//    }

	// first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
	// 一阶低通滤波代替斜波作为底盘速度输入
	//    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);

	// stop command, need not slow change, set zero derectly
	// 停止信号，不需要缓慢加速，直接减速到零
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
float K3_OUT = 0;
float K4_OUT = 0;
float K2_OUT = 0;
float K1_OUT = 0;
float TARGET_SPEED_POSITION;
float TARGET_SPEED_POSITION_V2;

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
	fp32 max_torque = 0.0f, torque_rate = 0.0f;
	fp32 temp = 0.0f;
	uint8_t i = 0;
	/*驱动轮输出力矩=SUM[LQR增益系数*(状态变量目标值-状态变量反馈值)]*/
	/*注意左右轮输出力矩的正负号，以及各状态变量反馈值的正负号*/
/*
	if (R_Y < 30)
	{

		// 左轮输出力矩
		Nm_R_test = -(
			LQR_K1 * (chassis_move_control_loop->chassis_position_tg - chassis_move_control_loop->chassis_position_now) +
			LQR_K2 * (chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) +
			LQR_K3 * chassis_move_control_loop->chassis_pitch -
			LQR_K4 * (-chassis_move_control_loop->chassis_pitch_speed));

		// 右轮输出力矩
		Nm_L_test = (LQR_K1 * (chassis_move_control_loop->chassis_position_tg - chassis_move_control_loop->chassis_position_now) +
					 LQR_K2 * (chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) +
					 LQR_K3 * chassis_move_control_loop->chassis_pitch -
					 LQR_K4 * (-chassis_move_control_loop->chassis_pitch_speed));
	}
	else if (R_Y >= 30)
	{
		// 左轮输出力矩
		Nm_R_test = -(
			M_LQR_K2 * (chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) +
			M_LQR_K3 * chassis_move_control_loop->chassis_pitch -
			M_LQR_K4 * (-chassis_move_control_loop->chassis_pitch_speed)
			//	+
			//	LQR_K15*chassis_move_control_loop->delta_angle
			//	-
			//	LQR_K16*chassis_move_control_loop->chassis_yaw_speed
		);
		// K2_OUT=	LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set);
		// K3_OUT=	LQR_K3*chassis_move_control_loop->chassis_pitch;
		// K4_OUT=LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed) ;

		// 右轮输出力矩
		Nm_L_test = (M_LQR_K2 * (chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) +
					 M_LQR_K3 * chassis_move_control_loop->chassis_pitch -
					 M_LQR_K4 * (-chassis_move_control_loop->chassis_pitch_speed)
					 //	+
					 //	LQR_K25*chassis_move_control_loop->delta_angle
					 //	-
					 //	LQR_K26*chassis_move_control_loop->chassis_yaw_speed
		);
	}
*/
		// 左轮输出力矩
		Nm_R_test = -(
			0 * (chassis_move_control_loop->chassis_position_tg - chassis_move_control_loop->chassis_position_now) +
			LQR_K2_REAL_TIME * (chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) +
			LQR_K3_REAL_TIME_xx * chassis_move_control_loop->chassis_pitch -
			LQR_K4_REAL_TIME_xx * (-chassis_move_control_loop->chassis_pitch_speed));

		// 右轮输出力矩
		Nm_L_test = (0 * (chassis_move_control_loop->chassis_position_tg - chassis_move_control_loop->chassis_position_now) +
					 LQR_K2_REAL_TIME * (chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) +
					 LQR_K3_REAL_TIME_xx * chassis_move_control_loop->chassis_pitch -
					 LQR_K4_REAL_TIME_xx * (-chassis_move_control_loop->chassis_pitch_speed));
		
		K1_OUT=LQR_K1_REAL_TIME * (chassis_move_control_loop->chassis_position_tg - chassis_move_control_loop->chassis_position_now) ;
		K2_OUT=	LQR_K2_REAL_TIME*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set);
		 K3_OUT=	LQR_K3_REAL_TIME_xx*chassis_move_control_loop->chassis_pitch;
		 K4_OUT=LQR_K4_REAL_TIME_xx*(-chassis_move_control_loop->chassis_pitch_speed) ;
		
		
	// 计算轮子最大转矩，并限制其最大转矩
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

	// 赋值电流值

	send_to_L_test = (int16_t)(Nm_L_test / CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN);

	send_to_R_test = (int16_t)(Nm_R_test / CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN);

	if ((DR16.rc.s_left == 3 && DR16.rc.s_right == 3) || (DR16.rc.s_left == 1 && DR16.rc.s_right == 3))
	{
		send_to_tire_L = send_to_L_test; // 将LQR算出来的值赋给发送变量
		send_to_tire_R = send_to_R_test;

		if(open_CHASSIS_follow==0)
	{
		if (DR16.rc.ch0 != 0)
		{
			TARGET_angle_YAW = DJIC_IMU.total_yaw + DR16.rc.ch0 / 15.0;
		}
		if (abs(DR16.mouse.x) >= 1)
		{
			TARGET_angle_YAW = TARGET_angle_YAW + DR16.mouse.x / 700.0;
		}
	}
	if(open_CHASSIS_follow==1)
	{
		YAW_TG_by_gimbal();//底盘跟随	
		if(CHASSIS_follow_times<6000)//5000ms后开启底盘跟随云台
		{
			CHASSIS_follow_times++;
			
		if(CHASSIS_follow_times<3000)//3000ms内底盘跟随不开启
		{
			follow_angle_real_use=0;////底盘跟随云台不开启
		}
		if(CHASSIS_follow_times>5000)//5000ms后站起来
		{
		liftoff_mode_T=0;
		}				
		
		}
		else
		{
			
		}

	TARGET_angle_YAW=DJIC_IMU.total_yaw+ follow_angle_real_use;
	}
		TARGET_angle_speed_YAW = P_PID_bate(&change_direction_angle, TARGET_angle_YAW, DJIC_IMU.total_yaw);
		if (DR16.rc.ch2 != 0) // 拨轮控制,控制速度
		{
			TARGET_angle_speed_YAW = DR16.rc.ch2;
			follow_angle_real_use=0;////底盘跟随云台不开启

			TARGET_angle_YAW = DJIC_IMU.total_yaw;
		}
		else if(abs(vjoy_TEST.ch_AD)>2)//键盘控制旋转速度
		{
		TARGET_angle_speed_YAW=vjoy_TEST.ch_AD*6.6 ;
		follow_angle_real_use=0;////底盘跟随云台不开启
		TARGET_angle_YAW = DJIC_IMU.total_yaw;
		}
		send_to_tire_L += P_PID_bate(&change_direction_speed, TARGET_angle_speed_YAW, DJIC_IMU.Gyro_z);
		send_to_tire_R += P_PID_bate(&change_direction_speed, TARGET_angle_speed_YAW, DJIC_IMU.Gyro_z);
	}
	else
	{
		LQR_TARGET_position = milemeter_test.total_mile_truly_use;
		CHASSIS_follow_times=0;////底盘跟随云台开启时间清零
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
// void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
//{
//     first_order_filter_type->frame_period = frame_period;
//     first_order_filter_type->num[0] = num[0];
//     first_order_filter_type->input = 0.0f;
//     first_order_filter_type->out = 0.0f;
// }

/**
 * @brief          一阶低通滤波计算
 * @author         RM
 * @param[in]      一阶低通滤波结构体
 * @param[in]      间隔的时间，单位 s
 * @retval         返回空
 */
// void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
//{
//     first_order_filter_type->input = input;
//     first_order_filter_type->out =
//         first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
// }

// 循环限幅函数
// fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
//{
//     if (maxValue < minValue)
//     {
//         return Input;
//     }

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
float chassis_vx_real=0;
float chassis_speed_real=0;
float REAL_BALANCE_ANGLE=0.0;
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	//    if (chassis_move_update == NULL)
	//    {
	//        return;
	//    }

	uint8_t i = 0;
	for (i = 0; i < 2; i++)
	{
		// update motor speed
		// 更新电机轮轴位移速度
		chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * M3508s[2 + i].realSpeed;
		// i=0是右轮   i=1是左轮
		// update motor angular velocity
		// 更新电机输出轴角速度
		chassis_move_update->motor_chassis[i].omg = CHASSIS_MOTOR_RPM_TO_OMG_SEN * M3508s[2 + i].realSpeed;

		// update motor torque
		// 更新电机转矩
		chassis_move_update->motor_chassis[i].torque = CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN * M3508s[2 + i].realCurrent;
	}

	// calculate chassis euler angle
	// 计算底盘姿态角度
	chassis_move_update->chassis_yaw =
		rad_format((float)INS_angle[0]);
	//	if(DR16.rc.ch1==0)
	
	chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2]);//重心补偿到1.5度平衡
	
//	chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2] - angle_qhq_pi_R+REAL_BALANCE_ANGLE/180.0f*PI);//重心补偿到1.5度平衡
	//	else
	//	{
	//    chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2]);
	//	}
	//	    chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2]);

	chassis_move_update->chassis_roll = (float)INS_angle[1];

	// calculate chassis euler angular velocity
	// 计算底盘姿态角速度
	chassis_move_update->chassis_yaw_speed = INS_gyro[2];

	chassis_move_update->chassis_pitch_speed = INS_gyro[0] - pitch_speed_new;

	chassis_move_update->chassis_roll_speed = INS_gyro[1];
	// calculate chassis velocity and synthesis angular velocity
	// 计算底盘速度和合成轮角速度
	chassis_move_update->vx = ((chassis_move_update->motor_chassis[0].speed) - (chassis_move_update->motor_chassis[1].speed)) / 2;
chassis_vx_real=chassis_move_update->vx ;

chassis_move_update->omg = ((chassis_move_update->motor_chassis[0].omg) - (chassis_move_update->motor_chassis[1].omg)) / 2;

	chassis_move_update->chassis_position_now = encoderToDistance((M3508s[3].totalAngle - M3508s[2].totalAngle) / 2);
	LQRweiyi_text = chassis_move_update->chassis_position_now;
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

	// get movement control target-points, 获取运动控制目标值
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
	LQR_target_position();
	if (DR16.rc.s_right == 3 && DR16.rc.s_left == 3)
	{
		chassis_move_control->chassis_yaw_set = rad_format(angle_set);
		chassis_move_control->delta_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
	}
	if (DR16.rc.s_right == 3 && DR16.rc.s_left == 1)
	{
		chassis_move_control->chassis_yaw_set = rad_format(angle_set);
		chassis_move_control->delta_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
		chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
	}
	else
	{
		chassis_move_control->chassis_yaw_set = chassis_move_control->chassis_yaw;
		angle_set = rad_format((float)INS_angle[0]);
	}
	//		else if (chassis_move_control->chassis_mode == CHASSIS_DOWN_MODE)
	//		{
	//			chassis_move_control->chassis_yaw_set = rad_format(angle_set);
	//			chassis_move_control->delta_angle = chassis_move_control->chassis_yaw_set;
	//      chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
	//    }
}

////限幅函数
// fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
//{
//     if (Value < minValue)
//         return minValue;
//     else if (Value > maxValue)
//         return maxValue;
//     else
//         return Value;
// }

float pitch_cut_off_angle = 10.0f; // 截止倾角超过这个角度就没速度了
float speed_damping_p = 1;		   // 衰减系数

void get_speed_by_position_V1()
{

	if (fabs((double)DJIC_IMU.total_pitch) >= pitch_cut_off_angle)
	{
		speed_damping_p = 0; // 超过截止角度，衰减系数为0
	}
	else
	{
		speed_damping_p = 1.0f - (fabs((double)DJIC_IMU.total_pitch) / pitch_cut_off_angle);
	}

	TARGET_SPEED_POSITION = P_PID_bate(&LQR_SPEED_BY_POSITION, LQR_TARGET_position, milemeter_test.total_mile_truly_use) / 1000.0f * speed_damping_p;

	if (DR16.rc.ch3 != 0) // 前进时更新目标位置
	{
		LQR_TARGET_position = milemeter_test.total_mile_truly_use;
		TARGET_SPEED_POSITION = 0;
	}
	if (DR16.rc.ch0 != 0) // 转向时更新目标位置
	{
		LQR_TARGET_position = milemeter_test.total_mile_truly_use;
		TARGET_SPEED_POSITION = 0;
	}
	if (DW_FOR_ZX != 0) // 小陀螺时更新目标位置
	{
		LQR_TARGET_position = milemeter_test.total_mile_truly_use;
		TARGET_SPEED_POSITION = 0;
	}
}

/*

float get_target_velocity(float target_position, float current_position, float dt)
{
	float error = target_position - current_position; // 计算误差
	float target_velocity = error / dt; // 计算目标速度
	return target_velocity; // 返回目标速度
}

float target_velocity(float target_pos, float current_pos, float delta_t) {
	float max_velocity = 1.5; // 最大速度为1.5m/s
	float velocity = (target_pos - current_pos) / delta_t; // 计算目标速度
	if (velocity > max_velocity) { // 如果速度超过了最大速度
		velocity = max_velocity; // 将速度限制为最大速度
	} else if (velocity < -max_velocity) { // 如果速度小于负的最大速度
		velocity = -max_velocity; // 将速度限制为负的最大速度
	}
	return velocity; // 返回限制后的速度
}




*/

// 减速比
// double reductionRatio = 3591.0 / 187.0;
//// 编码器分度值
// int encoderResolution = 8191;
//// 轮胎直径（单位：米）
// double wheelDiameter = 0.16;

// 将编码器转过的角度转换成轮胎滚动距离（单位：米）
double encoderToDistance(int encoderCount)
{
	// 计算轮胎周长
	//    double wheelCircumference = PI * wheelDiameter;
	//
	//    // 计算电机轴转过的角度
	//    double motorAngle = encoderCount / (double)encoderResolution * 2 * PI;
	//
	//	// 计算轮胎转过的角度
	//    double wheelAngle = motorAngle / reductionRatio;
	//
	//	// 计算轮胎滚动距离
	double distance = encoderCount * PI / 983073.58125;

	return distance;
}

void LQR_target_position()
{
	static int DR16_rc_ch3_last_V2;

	if (DR16.rc.s_right == 3 && DR16.rc.s_left == 3)
	{
		chassis_move.chassis_position_tg = chassis_move.chassis_position_tg + DR16.rc.ch3 / 660.0 / 1000.0f;
		LQRweiyi_PO_TG = chassis_move.chassis_position_tg;

		if (DR16.rc.ch3 == 0 && DR16_rc_ch3_last_V2 != 0)
		{
//			chassis_move.chassis_position_tg = chassis_move.chassis_position_now; // 记录松手瞬间的位置 作为目标位置
			chassis_move.chassis_position_tg = chassis_move.chassis_position_tg; // 记录松手瞬间的目标位置 位置 作为目标位置
		}

		DR16_rc_ch3_last_V2 = DR16.rc.ch3;
		if ((chassis_move.chassis_position_tg - chassis_move.chassis_position_now) > 1) // 目标如果距离当前位置一米以上，则限制
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now + 1;
		}
		if ((chassis_move.chassis_position_tg - chassis_move.chassis_position_now) < -1)
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now - 1;
		}

		if (fabs(DJIC_IMU.total_pitch) > 12.0f)
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now; // 为平衡时将 当前位置 作为目标位置
		}
	}
	else if (DR16.rc.s_right == 3 && DR16.rc.s_left == 1)
	{
		chassis_move.chassis_position_tg = chassis_move.chassis_position_tg + DR16.rc.ch3 / 660.0 / 1000.0f + vjoy_TEST.ch_WS / 100000.0;
		LQRweiyi_PO_TG = chassis_move.chassis_position_tg;

		if ((DR16.rc.ch1 == 0 && DR16_rc_ch3_last_V2 != 0) || (vjoy_TEST.ch_WS == 0 && DR16_rc_ch3_last_V2 != 0))
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now; // 记录松手瞬间的位置 作为目标位置
		}
		if(DR16.rc.ch0!=0)
		{
		chassis_move.chassis_position_tg = chassis_move.chassis_position_now; // 左右扭头自旋时的当前位置 作为目标位置
			//也可以根据云台YAW轴6020的角度判断
		}
		DR16_rc_ch3_last_V2 = DR16.rc.ch3 + vjoy_TEST.ch_WS;
		if ((chassis_move.chassis_position_tg - chassis_move.chassis_position_now) > 1) // 目标如果距离当前位置一米以上，则限制
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now + 1;
		}
		if ((chassis_move.chassis_position_tg - chassis_move.chassis_position_now) < -1)
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now - 1;
		}

		if (fabs(DJIC_IMU.total_pitch-angle_qhq_R) > 5.0f)//如果不靠摆杆补偿，实际的倾角大于5度
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now; // 为平衡时将 当前位置 作为目标位置
		}
	}
	else
	{
		chassis_move.chassis_position_tg = chassis_move.chassis_position_now;
	}
}

void get_speed_by_position_V2()
{
//	TARGET_SPEED_POSITION_V2 = DR16.rc.ch3 / -660.0;
	if (DR16.rc.s_right == 3 )//右中
	{
		if(DR16.rc.ch3!=0)
			TARGET_SPEED_POSITION_V2 = DR16.rc.ch3 / -330.0;//优先遥控器控制
		else if(abs(vjoy_TEST.ch_WS)>=2)
		TARGET_SPEED_POSITION_V2 = vjoy_TEST.ch_WS / -50.0f;
		else 
		TARGET_SPEED_POSITION_V2=0;
	}
	LQRweiyi_SPEED_TG = TARGET_SPEED_POSITION_V2;
	if (TARGET_SPEED_POSITION_V2 > 0.8f)
	{
		TARGET_SPEED_POSITION_V2 = 0.8;
	}
	if (TARGET_SPEED_POSITION_V2 < -0.8f)
	{
		TARGET_SPEED_POSITION_V2 = -0.8;
	}
	// if(fabs((double)DJIC_IMU.total_pitch)>=pitch_cut_off_angle)
	//{
	// speed_damping_p=0;//超过截止角度，衰减系数为0
	// }
	// else
	//{
	// speed_damping_p=1.0f-(fabs((double)DJIC_IMU.total_pitch)/pitch_cut_off_angle);
	// }

	// 	TARGET_SPEED_POSITION=P_PID_bate(&LQR_SPEED_BY_POSITION,LQR_TARGET_position,milemeter_test.total_mile_truly_use)/1000.0f*speed_damping_p;

	//	if(DR16.rc.ch1!=0)//前进时更新目标位置
	//{
	// LQR_TARGET_position=milemeter_test.total_mile_truly_use;
	// TARGET_SPEED_POSITION=0;
	//}
	//	if(DR16.rc.ch0!=0)//转向时更新目标位置
	//{
	// LQR_TARGET_position=milemeter_test.total_mile_truly_use;
	// TARGET_SPEED_POSITION=0;
	//}
	//	if(DW_FOR_ZX!=0)//小陀螺时更新目标位置
	//{
	// LQR_TARGET_position=milemeter_test.total_mile_truly_use;
	// TARGET_SPEED_POSITION=0;
	//}
}

double swing_link_length;//摆杆长度-实际
double swing_link_length_text;//摆杆长度-实际

void LQR_parameter()
{
	double x;
	x=swing_link_length*10.0f;
if(x<120)
x=120.0;
if(x>450)
x=450.0;//x取值的限制

LQR_K1_REAL_TIME=0;
LQR_K2_REAL_TIME=  (1E-10) * x * x * x - (1E-07) * x * x + (5E-05) * x - 3.1811;
//LQR_K3_REAL_TIME= (1E-05)*x* x - 0.0252*x - 6.8288;//浮点运算精度不够了
//LQR_K4_REAL_TIME= -1.0*4E-06*x*x - 0.0023*x - 1.3267;	
	
LQR_K3_REAL_TIME_xx=-0.015*x - 11.983;
LQR_K4_REAL_TIME_xx=-0.0042*x - 1.9927;	
/*
*/

/*
LQR_K1_REAL_TIME=0;
LQR_K2_REAL_TIME= -3.1781;
//LQR_K3_REAL_TIME= (1E-05)*x* x - 0.0252*x - 6.8288;//浮点运算精度不够了
//LQR_K4_REAL_TIME= -1.0*4E-06*x*x - 0.0023*x - 1.3267;	
	
LQR_K3_REAL_TIME_xx=-13.3144;
LQR_K4_REAL_TIME_xx=-2.7017;	

//
-0.707106781
-1.689469977
-7.38503189
-1.188470706

-1.22474487139155
-1.655190079
-7.138504982
-1.139992716

-0.707106781
-1.178686062
-6.519553148
-1.074337873

-0.02236068
-2.255149917
-12.84257646
-2.935861417

-3.1781
-13.3144
-2.7017
*/
}














