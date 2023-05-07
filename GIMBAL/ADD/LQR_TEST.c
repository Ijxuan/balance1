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

// �����˶�����
chassis_move_t chassis_move;
float LQRweiyi_text = 0;
float LQRweiyi_PO_TG = 0;	 // lqrλ��Ŀ��
float LQRweiyi_SPEED_TG = 0; // LQR�ٶ�Ŀ��

double LQR_K1_REAL_TIME;
double LQR_K2_REAL_TIME;
double LQR_K3_REAL_TIME;//����ʽ���
double LQR_K4_REAL_TIME;//����ʽ���
double LQR_K3_REAL_TIME_xx;//�������
double LQR_K4_REAL_TIME_xx;//�������

void LQR_TEST_CON()
{

	// chassis data update
	// �������ݸ���
	chassis_feedback_update(&chassis_move);

	// set chassis control set-point
	// ���̿���������
	chassis_set_contorl(&chassis_move);

	//LQR����ˢ��
	LQR_parameter();

	// chassis control LQR calculate
	// ���̿���LQR����
	chassis_control_loop(&chassis_move);
}

/**
 * @brief          ���ÿ���Ŀ����.���ݲ�ͬ���̿���ģʽ������ò�ͬ�Ŀ��ƺ���.
 * @param[out]     vx_set, ͨ�����������ƶ�.
 * @param[out]     angle_set, ͨ��������ת�˶�.
 * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
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
		chassis_remote_control(vx_set, angle_set, chassis_move_rc_to_vector); // ң��������ģʽ

		//			chassis_balance_control(vx_set, angle_set, chassis_move_rc_to_vector);//ƽ��ģʽ
	}
	if (DR16.rc.s_right == 2)
	{
		//			chassis_down_control(vx_set, angle_set, chassis_move_rc_to_vector);
	}
	if (DR16.rc.s_right == 3 && DR16.rc.s_left == 1)
	{
		chassis_remote_control(vx_set, angle_set, chassis_move_rc_to_vector); // ң��������ģʽ

		//			chassis_balance_control(vx_set, angle_set, chassis_move_rc_to_vector);//ƽ��ģʽ
	}

	//    else if (chassis_behaviour_mode == CHASSIS_REMOTE)
	//    {
	//			chassis_remote_control(vx_set, angle_set, chassis_move_rc_to_vector);
	//    }
}

/**
 * @brief          ����ң�ص���Ϊ״̬���£�Ŀ���ٶ���ҡ��3ͨ����ӳ�䣬Ŀ��ת����ҡ��2ͨ����ӳ��
 * @param[in]      vx_set ǰ�����ٶ�,��ֵ��ʾǰ���ٶȣ���ֵ��ʾ�����ٶ�
 * @param[in]      angle_set Ŀ��ת�ǣ���Χ-PI��PI
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
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
 * @brief          ����ң����ͨ��ֵ�����������ٶ��趨ֵ
 *
 * @param[out]     vx_set: �����ٶ�ָ��
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
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
	// �������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
	//    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);

	vx_channel = DR16.rc.ch3 * -1;
	vx_channel = 0;

	get_speed_by_position_V1(); // ����TARGET_SPEED_POSITION
	TARGET_SPEED_POSITION = 0;	// LQR��λ�û��������ٶȷ�����
	get_speed_by_position_V2();
	// ��ң�˲���ת��Ϊ�˶�����
	vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN + TARGET_SPEED_POSITION_V2;

	// keyboard set speed set-point
	// ���̿���
	//    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
	//    {
	//        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
	//    }
	//    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
	//    {
	//        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
	//    }

	// first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
	// һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
	//    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);

	// stop command, need not slow change, set zero derectly
	// ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
	//    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	//    {
	//        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	//    }

	*vx_set = vx_set_channel;
}

/**
 * @brief          ����DOWN������Ϊ״̬���£�Ŀ���ٶȺ�Ŀ��ת�Ǿ�Ϊ0
 * @param[in]      vx_set ǰ�����ٶ�,��ֵ��ʾǰ���ٶȣ���ֵ��ʾ�����ٶ�
 * @param[in]      angle_set Ŀ��ת��
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
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
 * @brief          ����ƽ�����Ϊ״̬���£�Ŀ���ٶȺ�Ŀ��ת�Ǿ�Ϊ0
 * @param[in]      vx_set ǰ�����ٶ�,��ֵ��ʾǰ���ٶȣ���ֵ��ʾ�����ٶ�
 * @param[in]      angle_set Ŀ��ת��
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
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
	/*�������������=SUM[LQR����ϵ��*(״̬����Ŀ��ֵ-״̬��������ֵ)]*/
	/*ע��������������ص������ţ��Լ���״̬��������ֵ��������*/
/*
	if (R_Y < 30)
	{

		// �����������
		Nm_R_test = -(
			LQR_K1 * (chassis_move_control_loop->chassis_position_tg - chassis_move_control_loop->chassis_position_now) +
			LQR_K2 * (chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) +
			LQR_K3 * chassis_move_control_loop->chassis_pitch -
			LQR_K4 * (-chassis_move_control_loop->chassis_pitch_speed));

		// �����������
		Nm_L_test = (LQR_K1 * (chassis_move_control_loop->chassis_position_tg - chassis_move_control_loop->chassis_position_now) +
					 LQR_K2 * (chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) +
					 LQR_K3 * chassis_move_control_loop->chassis_pitch -
					 LQR_K4 * (-chassis_move_control_loop->chassis_pitch_speed));
	}
	else if (R_Y >= 30)
	{
		// �����������
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

		// �����������
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
		// �����������
		Nm_R_test = -(
			0 * (chassis_move_control_loop->chassis_position_tg - chassis_move_control_loop->chassis_position_now) +
			LQR_K2_REAL_TIME * (chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) +
			LQR_K3_REAL_TIME_xx * chassis_move_control_loop->chassis_pitch -
			LQR_K4_REAL_TIME_xx * (-chassis_move_control_loop->chassis_pitch_speed));

		// �����������
		Nm_L_test = (0 * (chassis_move_control_loop->chassis_position_tg - chassis_move_control_loop->chassis_position_now) +
					 LQR_K2_REAL_TIME * (chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) +
					 LQR_K3_REAL_TIME_xx * chassis_move_control_loop->chassis_pitch -
					 LQR_K4_REAL_TIME_xx * (-chassis_move_control_loop->chassis_pitch_speed));
		
		K1_OUT=LQR_K1_REAL_TIME * (chassis_move_control_loop->chassis_position_tg - chassis_move_control_loop->chassis_position_now) ;
		K2_OUT=	LQR_K2_REAL_TIME*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set);
		 K3_OUT=	LQR_K3_REAL_TIME_xx*chassis_move_control_loop->chassis_pitch;
		 K4_OUT=LQR_K4_REAL_TIME_xx*(-chassis_move_control_loop->chassis_pitch_speed) ;
		
		
	// �����������ת�أ������������ת��
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

	// ��ֵ����ֵ

	send_to_L_test = (int16_t)(Nm_L_test / CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN);

	send_to_R_test = (int16_t)(Nm_R_test / CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN);

	if ((DR16.rc.s_left == 3 && DR16.rc.s_right == 3) || (DR16.rc.s_left == 1 && DR16.rc.s_right == 3))
	{
		send_to_tire_L = send_to_L_test; // ��LQR�������ֵ�������ͱ���
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
		YAW_TG_by_gimbal();//���̸���	
		if(CHASSIS_follow_times<6000)//5000ms�������̸�����̨
		{
			CHASSIS_follow_times++;
			
		if(CHASSIS_follow_times<3000)//3000ms�ڵ��̸��治����
		{
			follow_angle_real_use=0;////���̸�����̨������
		}
		if(CHASSIS_follow_times>5000)//5000ms��վ����
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
		if (DR16.rc.ch2 != 0) // ���ֿ���,�����ٶ�
		{
			TARGET_angle_speed_YAW = DR16.rc.ch2;
			follow_angle_real_use=0;////���̸�����̨������

			TARGET_angle_YAW = DJIC_IMU.total_yaw;
		}
		else if(abs(vjoy_TEST.ch_AD)>2)//���̿�����ת�ٶ�
		{
		TARGET_angle_speed_YAW=vjoy_TEST.ch_AD*6.6 ;
		follow_angle_real_use=0;////���̸�����̨������
		TARGET_angle_YAW = DJIC_IMU.total_yaw;
		}
		send_to_tire_L += P_PID_bate(&change_direction_speed, TARGET_angle_speed_YAW, DJIC_IMU.Gyro_z);
		send_to_tire_R += P_PID_bate(&change_direction_speed, TARGET_angle_speed_YAW, DJIC_IMU.Gyro_z);
	}
	else
	{
		LQR_TARGET_position = milemeter_test.total_mile_truly_use;
		CHASSIS_follow_times=0;////���̸�����̨����ʱ������
	}
}

/**
 * @brief          һ�׵�ͨ�˲���ʼ��
 * @author         RM
 * @param[in]      һ�׵�ͨ�˲��ṹ��
 * @param[in]      �����ʱ�䣬��λ s
 * @param[in]      �˲�����
 * @retval         ���ؿ�
 */
// void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
//{
//     first_order_filter_type->frame_period = frame_period;
//     first_order_filter_type->num[0] = num[0];
//     first_order_filter_type->input = 0.0f;
//     first_order_filter_type->out = 0.0f;
// }

/**
 * @brief          һ�׵�ͨ�˲�����
 * @author         RM
 * @param[in]      һ�׵�ͨ�˲��ṹ��
 * @param[in]      �����ʱ�䣬��λ s
 * @retval         ���ؿ�
 */
// void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
//{
//     first_order_filter_type->input = input;
//     first_order_filter_type->out =
//         first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
// }

// ѭ���޷�����
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
		// ���µ������λ���ٶ�
		chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * M3508s[2 + i].realSpeed;
		// i=0������   i=1������
		// update motor angular velocity
		// ���µ���������ٶ�
		chassis_move_update->motor_chassis[i].omg = CHASSIS_MOTOR_RPM_TO_OMG_SEN * M3508s[2 + i].realSpeed;

		// update motor torque
		// ���µ��ת��
		chassis_move_update->motor_chassis[i].torque = CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN * M3508s[2 + i].realCurrent;
	}

	// calculate chassis euler angle
	// ���������̬�Ƕ�
	chassis_move_update->chassis_yaw =
		rad_format((float)INS_angle[0]);
	//	if(DR16.rc.ch1==0)
	
	chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2]);//���Ĳ�����1.5��ƽ��
	
//	chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2] - angle_qhq_pi_R+REAL_BALANCE_ANGLE/180.0f*PI);//���Ĳ�����1.5��ƽ��
	//	else
	//	{
	//    chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2]);
	//	}
	//	    chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2]);

	chassis_move_update->chassis_roll = (float)INS_angle[1];

	// calculate chassis euler angular velocity
	// ���������̬���ٶ�
	chassis_move_update->chassis_yaw_speed = INS_gyro[2];

	chassis_move_update->chassis_pitch_speed = INS_gyro[0] - pitch_speed_new;

	chassis_move_update->chassis_roll_speed = INS_gyro[1];
	// calculate chassis velocity and synthesis angular velocity
	// ��������ٶȺͺϳ��ֽ��ٶ�
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
 * @brief          ���õ��̿���Ŀ��ֵ, �˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
	if (chassis_move_control == NULL)
	{
		return;
	}

	fp32 vx_set = 0.0f, angle_set = 0.0f;

	// get movement control target-points, ��ȡ�˶�����Ŀ��ֵ
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

////�޷�����
// fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
//{
//     if (Value < minValue)
//         return minValue;
//     else if (Value > maxValue)
//         return maxValue;
//     else
//         return Value;
// }

float pitch_cut_off_angle = 10.0f; // ��ֹ��ǳ�������ǶȾ�û�ٶ���
float speed_damping_p = 1;		   // ˥��ϵ��

void get_speed_by_position_V1()
{

	if (fabs((double)DJIC_IMU.total_pitch) >= pitch_cut_off_angle)
	{
		speed_damping_p = 0; // ������ֹ�Ƕȣ�˥��ϵ��Ϊ0
	}
	else
	{
		speed_damping_p = 1.0f - (fabs((double)DJIC_IMU.total_pitch) / pitch_cut_off_angle);
	}

	TARGET_SPEED_POSITION = P_PID_bate(&LQR_SPEED_BY_POSITION, LQR_TARGET_position, milemeter_test.total_mile_truly_use) / 1000.0f * speed_damping_p;

	if (DR16.rc.ch3 != 0) // ǰ��ʱ����Ŀ��λ��
	{
		LQR_TARGET_position = milemeter_test.total_mile_truly_use;
		TARGET_SPEED_POSITION = 0;
	}
	if (DR16.rc.ch0 != 0) // ת��ʱ����Ŀ��λ��
	{
		LQR_TARGET_position = milemeter_test.total_mile_truly_use;
		TARGET_SPEED_POSITION = 0;
	}
	if (DW_FOR_ZX != 0) // С����ʱ����Ŀ��λ��
	{
		LQR_TARGET_position = milemeter_test.total_mile_truly_use;
		TARGET_SPEED_POSITION = 0;
	}
}

/*

float get_target_velocity(float target_position, float current_position, float dt)
{
	float error = target_position - current_position; // �������
	float target_velocity = error / dt; // ����Ŀ���ٶ�
	return target_velocity; // ����Ŀ���ٶ�
}

float target_velocity(float target_pos, float current_pos, float delta_t) {
	float max_velocity = 1.5; // ����ٶ�Ϊ1.5m/s
	float velocity = (target_pos - current_pos) / delta_t; // ����Ŀ���ٶ�
	if (velocity > max_velocity) { // ����ٶȳ���������ٶ�
		velocity = max_velocity; // ���ٶ�����Ϊ����ٶ�
	} else if (velocity < -max_velocity) { // ����ٶ�С�ڸ�������ٶ�
		velocity = -max_velocity; // ���ٶ�����Ϊ��������ٶ�
	}
	return velocity; // �������ƺ���ٶ�
}




*/

// ���ٱ�
// double reductionRatio = 3591.0 / 187.0;
//// �������ֶ�ֵ
// int encoderResolution = 8191;
//// ��ֱ̥������λ���ף�
// double wheelDiameter = 0.16;

// ��������ת���ĽǶ�ת������̥�������루��λ���ף�
double encoderToDistance(int encoderCount)
{
	// ������̥�ܳ�
	//    double wheelCircumference = PI * wheelDiameter;
	//
	//    // ��������ת���ĽǶ�
	//    double motorAngle = encoderCount / (double)encoderResolution * 2 * PI;
	//
	//	// ������̥ת���ĽǶ�
	//    double wheelAngle = motorAngle / reductionRatio;
	//
	//	// ������̥��������
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
//			chassis_move.chassis_position_tg = chassis_move.chassis_position_now; // ��¼����˲���λ�� ��ΪĿ��λ��
			chassis_move.chassis_position_tg = chassis_move.chassis_position_tg; // ��¼����˲���Ŀ��λ�� λ�� ��ΪĿ��λ��
		}

		DR16_rc_ch3_last_V2 = DR16.rc.ch3;
		if ((chassis_move.chassis_position_tg - chassis_move.chassis_position_now) > 1) // Ŀ��������뵱ǰλ��һ�����ϣ�������
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now + 1;
		}
		if ((chassis_move.chassis_position_tg - chassis_move.chassis_position_now) < -1)
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now - 1;
		}

		if (fabs(DJIC_IMU.total_pitch) > 12.0f)
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now; // Ϊƽ��ʱ�� ��ǰλ�� ��ΪĿ��λ��
		}
	}
	else if (DR16.rc.s_right == 3 && DR16.rc.s_left == 1)
	{
		chassis_move.chassis_position_tg = chassis_move.chassis_position_tg + DR16.rc.ch3 / 660.0 / 1000.0f + vjoy_TEST.ch_WS / 100000.0;
		LQRweiyi_PO_TG = chassis_move.chassis_position_tg;

		if ((DR16.rc.ch1 == 0 && DR16_rc_ch3_last_V2 != 0) || (vjoy_TEST.ch_WS == 0 && DR16_rc_ch3_last_V2 != 0))
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now; // ��¼����˲���λ�� ��ΪĿ��λ��
		}
		if(DR16.rc.ch0!=0)
		{
		chassis_move.chassis_position_tg = chassis_move.chassis_position_now; // ����Ťͷ����ʱ�ĵ�ǰλ�� ��ΪĿ��λ��
			//Ҳ���Ը�����̨YAW��6020�ĽǶ��ж�
		}
		DR16_rc_ch3_last_V2 = DR16.rc.ch3 + vjoy_TEST.ch_WS;
		if ((chassis_move.chassis_position_tg - chassis_move.chassis_position_now) > 1) // Ŀ��������뵱ǰλ��һ�����ϣ�������
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now + 1;
		}
		if ((chassis_move.chassis_position_tg - chassis_move.chassis_position_now) < -1)
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now - 1;
		}

		if (fabs(DJIC_IMU.total_pitch-angle_qhq_R) > 5.0f)//��������ڸ˲�����ʵ�ʵ���Ǵ���5��
		{
			chassis_move.chassis_position_tg = chassis_move.chassis_position_now; // Ϊƽ��ʱ�� ��ǰλ�� ��ΪĿ��λ��
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
	if (DR16.rc.s_right == 3 )//����
	{
		if(DR16.rc.ch3!=0)
			TARGET_SPEED_POSITION_V2 = DR16.rc.ch3 / -330.0;//����ң��������
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
	// speed_damping_p=0;//������ֹ�Ƕȣ�˥��ϵ��Ϊ0
	// }
	// else
	//{
	// speed_damping_p=1.0f-(fabs((double)DJIC_IMU.total_pitch)/pitch_cut_off_angle);
	// }

	// 	TARGET_SPEED_POSITION=P_PID_bate(&LQR_SPEED_BY_POSITION,LQR_TARGET_position,milemeter_test.total_mile_truly_use)/1000.0f*speed_damping_p;

	//	if(DR16.rc.ch1!=0)//ǰ��ʱ����Ŀ��λ��
	//{
	// LQR_TARGET_position=milemeter_test.total_mile_truly_use;
	// TARGET_SPEED_POSITION=0;
	//}
	//	if(DR16.rc.ch0!=0)//ת��ʱ����Ŀ��λ��
	//{
	// LQR_TARGET_position=milemeter_test.total_mile_truly_use;
	// TARGET_SPEED_POSITION=0;
	//}
	//	if(DW_FOR_ZX!=0)//С����ʱ����Ŀ��λ��
	//{
	// LQR_TARGET_position=milemeter_test.total_mile_truly_use;
	// TARGET_SPEED_POSITION=0;
	//}
}

double swing_link_length;//�ڸ˳���-ʵ��
double swing_link_length_text;//�ڸ˳���-ʵ��

void LQR_parameter()
{
	double x;
	x=swing_link_length*10.0f;
if(x<120)
x=120.0;
if(x>450)
x=450.0;//xȡֵ������

LQR_K1_REAL_TIME=0;
LQR_K2_REAL_TIME=  (1E-10) * x * x * x - (1E-07) * x * x + (5E-05) * x - 3.1811;
//LQR_K3_REAL_TIME= (1E-05)*x* x - 0.0252*x - 6.8288;//�������㾫�Ȳ�����
//LQR_K4_REAL_TIME= -1.0*4E-06*x*x - 0.0023*x - 1.3267;	
	
LQR_K3_REAL_TIME_xx=-0.015*x - 11.983;
LQR_K4_REAL_TIME_xx=-0.0042*x - 1.9927;	
/*
*/

/*
LQR_K1_REAL_TIME=0;
LQR_K2_REAL_TIME= -3.1781;
//LQR_K3_REAL_TIME= (1E-05)*x* x - 0.0252*x - 6.8288;//�������㾫�Ȳ�����
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














