#include"LQR_TEST.h"
#include "DR16_RECIVE.h"
#include "M3508.h"
#include "INS_task.h"
#include "user_lib.h"

static void chassis_balance_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_remote_control(fp32 *vx_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

int send_to_L_test;
int send_to_R_test;

//�����˶�����
chassis_move_t chassis_move;

void LQR_TEST_CON()
{

        //chassis data update
        //�������ݸ���
        chassis_feedback_update(&chassis_move);
	
        //set chassis control set-point 
        //���̿���������
        chassis_set_contorl(&chassis_move);

	        //chassis control LQR calculate
        //���̿���LQR����
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

 if (DR16.rc.s_right == 3&& DR16.rc.s_left==3)
    {
			chassis_balance_control(vx_set, angle_set, chassis_move_rc_to_vector);//ƽ��ģʽ
    }
 if (DR16.rc.s_right == 2)
    {
//			chassis_down_control(vx_set, angle_set, chassis_move_rc_to_vector);
    }	
 if (DR16.rc.s_right == 1)
    {
//chassis_remote_control(vx_set, angle_set, chassis_move_rc_to_vector);//ң��������ģʽ
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
    if (vx_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;  //
    }

    chassis_rc_to_control_vector(vx_set, chassis_move_rc_to_vector);
		
    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - 
	CHASSIS_ANGLE_Z_RC_SEN * DR16.rc.ch0);
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
void chassis_rc_to_control_vector(fp32 *vx_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL )
    {
        return;
    }
    
    int16_t vx_channel ;
    fp32 vx_set_channel ;
		
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
//    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
		
	vx_channel=DR16.rc.ch1;
	
    //��ң�˲���ת��Ϊ�˶�����
    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
		
    //keyboard set speed set-point
    //���̿���
//    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
//    {
//        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
//    }
//    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
//    {
//        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
//    }

    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
//    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    
		//stop command, need not slow change, set zero derectly
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
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

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
		/*�������������=SUM[LQR����ϵ��*(״̬����Ŀ��ֵ-״̬��������ֵ)]*/
		/*ע��������������ص������ţ��Լ���״̬��������ֵ��������*/
	  //�����������
 send_to_L_test =
	- (LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) - 
	LQR_K3*chassis_move_control_loop->chassis_pitch  + 
	LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed) + 
	LQR_K15*chassis_move_control_loop->delta_angle + 
	LQR_K16*chassis_move_control_loop->chassis_yaw_speed);
	
		  //�����������

 send_to_R_test =   
	(LQR_K2*(chassis_move_control_loop->vx - chassis_move_control_loop->vx_set) - 
	LQR_K3*chassis_move_control_loop->chassis_pitch  + 
	LQR_K4*(-chassis_move_control_loop->chassis_pitch_speed) + 
	LQR_K25*chassis_move_control_loop->delta_angle + 
	LQR_K26*chassis_move_control_loop->chassis_yaw_speed);






}

















/**
  * @brief          һ�׵�ͨ�˲���ʼ��
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
//void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1])
//{
//    first_order_filter_type->frame_period = frame_period;
//    first_order_filter_type->num[0] = num[0];
//    first_order_filter_type->input = 0.0f;
//    first_order_filter_type->out = 0.0f;
//}


/**
  * @brief          һ�׵�ͨ�˲�����
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @retval         ���ؿ�
  */
//void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input)
//{
//    first_order_filter_type->input = input;
//    first_order_filter_type->out =
//        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
//}


//ѭ���޷�����
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
			//���µ������λ���ٶ�
			chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * M3508s[2+i].realSpeed;
			//i=0������   i=1������
			//update motor angular velocity
			//���µ���������ٶ�
			chassis_move_update->motor_chassis[i].omg = CHASSIS_MOTOR_RPM_TO_OMG_SEN * M3508s[2+i].realSpeed;
			
			//update motor torque
			//���µ��ת��
			chassis_move_update->motor_chassis[i].torque = CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN * M3508s[2+i].realCurrent;
    }

    //calculate chassis euler angle
    //���������̬�Ƕ�
    chassis_move_update->chassis_yaw = 
	rad_format((float)INS_angle[0]);
	
    chassis_move_update->chassis_pitch = rad_format((float)INS_angle[2]);
    chassis_move_update->chassis_roll = (float)INS_angle[1];
		
		//calculate chassis euler angular velocity
		//���������̬���ٶ�
		chassis_move_update->chassis_yaw_speed = INS_gyro[2];
		
	chassis_move_update->chassis_pitch_speed =  INS_gyro[0];
		
	chassis_move_update->chassis_roll_speed = INS_gyro[1];	  
		//calculate chassis velocity and synthesis angular velocity
		//��������ٶȺͺϳ��ֽ��ٶ�
		chassis_move_update->vx = ((chassis_move_update->motor_chassis[0].speed) - (chassis_move_update->motor_chassis[1].speed))/2 ;
	  chassis_move_update->omg = ((chassis_move_update->motor_chassis[0].omg) - (chassis_move_update->motor_chassis[1].omg))/2 ;
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
		
    //get movement control target-points, ��ȡ�˶�����Ŀ��ֵ
    chassis_behaviour_control_set(&vx_set, &angle_set, chassis_move_control);

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
			chassis_move_control->delta_angle = chassis_move_control->chassis_yaw_set;
			chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
	
//		else if (chassis_move_control->chassis_mode == CHASSIS_DOWN_MODE)
//		{
//			chassis_move_control->chassis_yaw_set = rad_format(angle_set);
//			chassis_move_control->delta_angle = chassis_move_control->chassis_yaw_set;
//      chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
//    }
}


////�޷�����
//fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
//{
//    if (Value < minValue)
//        return minValue;
//    else if (Value > maxValue)
//        return maxValue;
//    else
//        return Value;
//}

