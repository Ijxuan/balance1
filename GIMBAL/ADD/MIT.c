#include "MIT.h"
#include "DR16_RECIVE.h"
#include "my_positionPID_bate.h"
#include "math.h"
#include "MY_balance_CONTROL.h"
#include "mit_math.h"
#include "bsp_buzzer.h"
#include "M3508.h"
#include "LPF.h"

void MIT_PID_INIT(void)
{
	P_PID_Parameter_Init(&MIT_A_SPEED, 0.8, 0.03, -0.6, 4, //-0.00001
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 555, -555,
						 60, -60);					// //MIT��� �ٶȻ�
	P_PID_V2_Init(&MIT_A_POSITION, 4, 0.25, 0, 100, //-0.00001
				  //						  float max_error, float min_error,
				  //                          float alpha,
				  50, -50,
				  0.8, -0.8,
				  350, -350,
				  100, -100); // //MIT��� λ�û�

	P_PID_Parameter_Init(&MIT_B_SPEED, 0.8, 0.03, -0.6, 4, //-0.00001
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 555, -555,
						 60, -60);					// //MIT��� �ٶȻ�
	P_PID_V2_Init(&MIT_B_POSITION, 4, 0.25, 0, 100, //-0.00001
				  //						  float max_error, float min_error,
				  //                          float alpha,
				  50, -50,
				  0.8, -0.8,
				  350, -350,
				  100, -100); // //MIT��� λ�û�

	P_PID_Parameter_Init(&MIT_C_SPEED, 0.8, 0.03, -0.6, 4, //-0.00001
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 555, -555,
						 60, -60);					// //MIT��� �ٶȻ�
	P_PID_V2_Init(&MIT_C_POSITION, 4, 0.25, 0, 100, //-0.00001
				  //						  float max_error, float min_error,
				  //                          float alpha,
				  50, -50,
				  0.8, -0.8,
				  350, -350,
				  100, -100); // //MIT��� λ�û�

	P_PID_Parameter_Init(&MIT_D_SPEED, 0.8, 0.03, -0.6, 4, //-0.00001
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 555, -555,
						 60, -60);					// //MIT��� �ٶȻ�
	P_PID_V2_Init(&MIT_D_POSITION, 4, 0.25, 0, 100, //-0.00001
				  //						  float max_error, float min_error,
				  //                          float alpha,
				  50, -50,
				  0.8, -0.8,
				  350, -350,
				  100, -100); // //MIT��� λ�û�

	SEND_TO_MIT_MAX.Rate = 0.15;
	SEND_TO_MIT_MAX.Absolute_Max = 80;
	MIT_A.kp_temp = 50;
	MIT_A.kv_temp = 3;

	MIT_B.kp_temp = 50;
	MIT_B.kv_temp = 3;

	MIT_C.kp_temp = 50;
	MIT_C.kv_temp = 3;

	MIT_D.kp_temp = 50;
	MIT_D.kv_temp = 3;

	liftoff_temp.Rate = 0.1;

	MIT_BALENCE_start.Absolute_Max = 20; // ͨ��MIT���ֻ���ƽ�� б�º�������ֵ
	MIT_BALENCE_start.Rate = 0.002;

	MIT_BALENCE_GO_TO_TG.Absolute_Max = 20;
	MIT_BALENCE_GO_TO_TG.Rate = 0.02;

	liftoff_SE.Absolute_Max = 43;
	liftoff_SE.Rate = 0.01;
}

int MIT_MODE_TEXT = 1;
MIT_t text_moto;
MIT_t MIT_A;
MIT_t MIT_B;
MIT_t MIT_C;
MIT_t MIT_D;

float liftoff_R = 10;		// �ұ���ظ߶�
float liftoff_L = 10;		// �����ظ߶�
float engine_body_height_R; // �ұ߻���߶�
float engine_body_height_L; // ��߻���߶�
int16_t sendto_MIT_TEXT = 1;
float send_to_MIT_text = 0; // ���͸������ֵ
/**/
int MIT_ANGLE_JD_LAST;		// ��һʱ�̵���Ƕ�
int MIT_ANGLE_JD_LAST_LAST; // ��һʱ�̵���Ƕ�

int MIT_ANGLE_JD_CHANGE; // ����ʱ�̵���Ƕȵı仯ֵ

int MIT_SPEED_BY_ANGLE;		 // �������νǶ�֮��������ٶ�
int MIT_SPEED_BY_ANGLE_TEMP; // �������νǶ�֮��������ٶ� ��ʱ

int MIT_SPEED_NEW; // ��ʱ

int i_for_speed = 1; // ��ü���һ���ٶ�

int L_OR_R = 0;

float send_to_MIT_damping = 0; // ���͸������ֵ��˥��

int MIT_DISABLE_TIMES = 0; // ���ʧ��ʱ���ۼ�
int MIT_ENABLE_TIMES = 0;  // ���ʹ��ʱ���ۼ�

float L_C_X_NOW = 0; // ��ʱ�˿����ֵ����꣨x�ᣩ
float L_C_Y_NOW = 0; // ��ʱ�˿����ֵ����꣨y�ᣩ

float R_C_X_NOW = 0; // ��ʱ�˿����ֵ����꣨x�ᣩ
float R_C_Y_NOW = 0; // ��ʱ�˿����ֵ����꣨y�ᣩ
/* ��buf�е�����ͨ��CAN�ӿڷ��ͳ�ȥ */
static void CanTransmit(uint8_t *buf, uint8_t len, uint32_t id)
{
	CAN_TxHeaderTypeDef TxHead; /**!< canͨ�ŷ���Э��ͷ */
	uint32_t canTxMailbox;

	if ((buf != NULL) && (len != 0))
	{
		TxHead.StdId = id;		   /* ָ����׼��ʶ������ֵ��0x00-0x7FF */
		TxHead.IDE = CAN_ID_STD;   /* ָ����Ҫ������Ϣ�ı�ʶ������ */
		TxHead.RTR = CAN_RTR_DATA; /* ָ����Ϣ����֡���� */
		TxHead.DLC = len;		   /* ָ����Ҫ�����֡���� */

		if (HAL_CAN_AddTxMessage(&hcan2, &TxHead, buf, (uint32_t *)&canTxMailbox) == HAL_OK)
		{
		}
	}
}

void MIT_MODE(uint8_t MODE)
{
	uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
	switch (MODE)
	{
	case CMD_MOTOR_MODE:
		buf[7] = 0xFC;
		break;

	case CMD_RESET_MODE:
		buf[7] = 0xFD;
		break;

	case CMD_ZERO_POSITION:
		buf[7] = 0xFE;
		break;

	default:
		return; /* ֱ���˳����� */
	}
	CAN_SendData(&hcan2, CAN_ID_STD, TEST_MIT_SLAVE_ID, buf);
	//	CanTransmit(buf,6,TEST_MIT_SLAVE_ID);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)

{
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
/**
 * @brief  Converts a float to an unsigned int, given range and number of bits
 * @param
 * @retval
 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
	float span = x_max - x_min;
	float offset = x_min;

	return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
  * @brief  Can���߷��Ϳ��Ʋ���
  * @param
  * @retval

  */
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t, uint32_t id)
{
	uint16_t p, v, kp, kd, t;
	uint8_t buf[8];

	/* ��������Ĳ����ڶ���ķ�Χ�� */
	LIMIT_MIN_MAX(f_p, P_MIN, P_MAX);
	LIMIT_MIN_MAX(f_v, V_MIN, V_MAX);
	LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
	LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
	LIMIT_MIN_MAX(f_t, T_MIN, T_MAX);

	/* ����Э�飬��float��������ת�� */
	p = float_to_uint(f_p, P_MIN, P_MAX, 16);
	v = float_to_uint(f_v, V_MIN, V_MAX, 12);
	kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
	kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
	t = float_to_uint(f_t, T_MIN, T_MAX, 12);

	/* ���ݴ���Э�飬������ת��ΪCAN���������ֶ� */
	buf[0] = p >> 8;
	buf[1] = p & 0xFF;
	buf[2] = v >> 4;
	buf[3] = ((v & 0xF) << 4) | (kp >> 8);
	buf[4] = kp & 0xFF;
	buf[5] = kd >> 4;
	buf[6] = ((kd & 0xF) << 4) | (t >> 8);
	buf[7] = t & 0xff;

	/* ͨ��CAN�ӿڰ�buf�е����ݷ��ͳ�ȥ */
	//    CanTransmit(buf, sizeof(buf));

	CAN_SendData(&hcan2, CAN_ID_STD, id, buf);
}

float position_text = 0;	  // Ŀ��Ƕ�
float position_text_TEMP = 0; // Ŀ��Ƕ�

float position_HD_text = 0; // Ŀ��Ƕ�-������

float speed_text = 0;	 // Ŀ���ٶ�
float speed_HD_text = 0; // Ŀ���ٶ�-������

float kp_text = 0; // �Ƕ�ϵ�� 5  3
float kv_text = 0; // �ٶ�ϵ�� 1  1
float NJ_text = 0; // Ŀ��Ť��

float MAX_OUT = 0; // ������

Ramp_Struct MIT_P; // Ŀ��λ��б��

Ramp_Struct SEND_TO_MIT_MAX; //

float L_X = 0; // ��xĿ��λ��
float L_Y = 0; // ��YĿ��λ��
float R_X = 0; // ��xĿ��λ��
float R_Y = 0; // ��YĿ��λ��

int disable_position;
int DR16_rc_ch1_last_for_mit;

int R_speed_new_FOR_MIT;
int L_speed_new_FOR_MIT;

void MIT_controul(void)
{

	// position_text+=DR16.rc.ch1/1100.0f;
	//	speed_HD_text=DR16.rc.ch3/18.0f;//ң�����ٶȸ����ȵ�Ŀ��ֵ+-35

	// if(position_text_TEMP>-1)position_text_TEMP=-1;
	//
	// if(position_text_TEMP<-90)position_text_TEMP=-90;
	// MIT_P.Target_Value=position_text_TEMP;//б��Ŀ��ֵ
	//	MIT_P.Current_Value=position_text;//б�µ�ǰֵ
	//	position_text=Ramp_Function(&MIT_P);

	//	MIT_B_SPEED.Max_result=MAX_OUT;
	//	MIT_A_SPEED.Max_result=MAX_OUT;
	//	MIT_C_SPEED.Max_result=MAX_OUT;
	//	MIT_D_SPEED.Max_result=MAX_OUT;
	//
	//	MIT_B_SPEED.Min_result=-MAX_OUT;
	//	MIT_A_SPEED.Min_result=-MAX_OUT;
	//	MIT_C_SPEED.Min_result=-MAX_OUT;
	//	MIT_D_SPEED.Min_result=-MAX_OUT;

	// if(position_text>-1)position_text=-1;
	//
	// if(position_text<-90)position_text=-90;
	//
	//	position_HD_text=position_text/Angle_turn_Radian;

	//	speed_HD_text=P_PID_bate(&MIT_TEXT,position_HD_text,text_moto.position_end);//�û�������PID�ջ�

	//	speed_text=speed_HD_text*Angle_turn_Radian;//Ŀ���ٶ�ת�Ƕȷ���۲�
	//
	target_position_text_PID += DR16.rc.ch1 / 1100.0f;

	// MIT_P.Target_Value=target_position_text_PID+DR16.rc.ch1/1100.0f;//б��Ŀ��ֵ
	//	MIT_P.Current_Value=target_position_text_PID;//б�µ�ǰֵ
	//	target_position_text_PID=Ramp_Function(&MIT_P);
	if (target_position_text_PID > 90)
		target_position_text_PID = 90;

	if (target_position_text_PID < -1)
		target_position_text_PID = -1;
	//	speed_text_v();
	// if(text_moto.ANGLE_JD>-2)send_to_MIT_text=0;
	//
	// if(text_moto.ANGLE_JD<-89)send_to_MIT_text=0;

	// CanComm_SendControlPara(position_HD_text,speed_HD_text,kp_text,kv_text,send_to_MIT_text);
	// CanComm_SendControlPara(position_HD_text,speed_HD_text,0,0,0,MIT_A_SLAVE_ID);
	// CanComm_SendControlPara(position_HD_text,speed_HD_text,0,0,0,MIT_B_SLAVE_ID);
	L_OR_R++;

	// if(MIT_C.ANGLE_JD<MIT_C.MIT_TZG_ARRIVE)
	//{
	// MIT_C.MIT_TZG_ARRIVE=MIT_C.ANGLE_JD;//ˢ�±߽�ֵ
	// }
	// if(MIT_D.ANGLE_JD>MIT_D.MIT_TZG_ARRIVE)
	//{
	// MIT_D.MIT_TZG_ARRIVE=MIT_D.ANGLE_JD;//ˢ�±߽�ֵ
	// }
	// if(MIT_A.ANGLE_JD<MIT_A.MIT_TZG_ARRIVE)
	//{
	// MIT_A.MIT_TZG_ARRIVE=MIT_A.ANGLE_JD;//ˢ�±߽�ֵ
	// }
	// if(MIT_B.ANGLE_JD>MIT_B.MIT_TZG_ARRIVE)
	//{
	// MIT_B.MIT_TZG_ARRIVE=MIT_B.ANGLE_JD;//ˢ�±߽�ֵ
	// }

	if (DR16.rc.s_right == 2)
	{
		L_X = 10;
		R_X = 10;
		// MIT_change_focus.result=0;
		disable_position = milemeter_test.total_mile_by_angle_100; // ʧ��ʱ
	}
	else if (DR16.rc.s_right == 3)
	{
		if (DR16.rc.ch1 != 0)
		{
			disable_position = milemeter_test.total_mile_by_angle_100 + DR16.rc.ch1 / 3; // ң��������Ŀ��λ��

			// disable_position=milemeter_test.total_mile_truly_use+P_PID_bate(&RC_SPEED_TO_POSITION,TARGET_speed_RC,L_speed_new-R_speed_new);//ң��������Ŀ���ٶ�,ת�����ٶ�
		}

		if (DR16.rc.ch1 == 0 && DR16_rc_ch1_last_for_mit != 0)
		{
			disable_position = milemeter_test.total_mile_by_angle_100; // ��¼����˲���λ��
		}
		DR16_rc_ch1_last_for_mit = DR16.rc.ch1;

		P_PID_bate_V2(&MIT_change_focus,
					  disable_position,
					  milemeter_test.total_mile_by_angle_100);

		L_speed_new_FOR_MIT = LPF_V2(&SPEED_L_FOR_MIT, M3508s[3].realSpeed / 100);
		R_speed_new_FOR_MIT = LPF_V2(&SPEED_R_FOR_MIT, M3508s[2].realSpeed / 100);

		P_PID_bate_V2(&MIT_change_focus_by_speed,
					  MIT_change_focus.result / 100,
					  (L_speed_new_FOR_MIT - R_speed_new_FOR_MIT));
	}

	// get_MIT_tg_angle_for_liftoff();//��������ظ߶�

	// get_MIT_tg_angle_for_bais();//������ǰ����б�Ƕ�

	Accurately_contrul_text();

	if (L_OR_R % 2 == 0)
	{

		mit_math_temp_2(R_X, R_Y); ///*ƽ�����������*/
		get_tg_angle_by_WLG_IS();

		MIT_B_controul();
		MIT_A_controul();
	}
	else
	{
		// liftoff_R+=DR16.rc.ch3/2200.0f;	//��������
		// liftoff_L+=DR16.rc.ch3/2200.0f;	//��������

		// if(liftoff_L>90)liftoff_L=90;
		// if(liftoff_L<1)liftoff_L=1;
		mit_math_temp_2(L_X, L_Y); ///*ƽ�����������*/
		get_tg_angle_by_WLG_IS();

		MIT_C_controul();
		MIT_D_controul();
	}
	/*
	float torque_ref = controller->kp*(controller->p_des - controller->theta_mech) + controller->t_ff + controller->kd*(controller->v_des - controller->dtheta_mech);
	*/
}

int speed_add_or_fall;				 // �ٶ��������Ǽ�С; 1������ 2�Ǽ�С
float target_speed_text = 0;		 // ������Ŀ���ٶ�
float target_speed_text_value = 80;	 // ������Ŀ���ٶ���ֵ,����Ϊ��ֵ
float target_position_text_PID = -1; // PID������Ŀ��λ��

void speed_text_v(void)
{

	// if(text_moto.ANGLE_JD>-10)//
	//{
	// speed_add_or_fall=2;//��ʼ��С
	// }
	// if(text_moto.ANGLE_JD<-80)//
	//{
	// speed_add_or_fall=1;//��ʼ����
	// }
	// target_speed_text_value=fabs(target_speed_text_value);
	// if(speed_add_or_fall==1)
	//{
	// target_speed_text=target_speed_text_value;//���ٶ�
	// }
	// if(speed_add_or_fall==2)
	//{
	// target_speed_text=-target_speed_text_value;//���ٶ�
	// }

	target_speed_text = P_PID_bate(&MIT_POSITION_TEXT, target_position_text_PID, text_moto.ANGLE_JD);
	if (text_moto.ANGLE_JD > 93)
		target_speed_text = 0;

	if (text_moto.ANGLE_JD < -5)
		target_speed_text = 0;

	send_to_MIT_text = P_PID_bate(&MIT_SPEED_TEXT, target_speed_text, text_moto.SPEED_JD) / 10.0f;
}

void MIT_B_controul(void)
{
/*
liftoff_R+=DR16.rc.ch3/2200.0f;	//��������
if(liftoff_R>90)liftoff_R=90;

if(liftoff_R<10)liftoff_R=10;
liftoff_L=liftoff_R;
MIT_B.target_position=liftoff_R-125.9;//����ֱ��-125.9�� ����һ����ֵ(liftoff_R)

if(MIT_B.target_position>-30)MIT_B.target_position=-30;

if(MIT_B.target_position<-120)MIT_B.target_position=-120;


MIT_B_SPEED.Target=P_PID_bate(&MIT_B_POSITION,MIT_B.target_position,MIT_B.ANGLE_JD);



MIT_B.send_to_MIT=P_PID_bate(&MIT_B_SPEED,MIT_B_SPEED.Target,MIT_B.SPEED_JD)/10.0f;

if(MIT_B.ANGLE_JD>-25.1)MIT_B.send_to_MIT=0;

if(MIT_B.ANGLE_JD<-126.2)MIT_B.send_to_MIT=0;
//���γ���0.3��ʧ��
//-25.6  -125.9
CanComm_SendControlPara(0,0,0,0,MIT_B.send_to_MIT,MIT_B_SLAVE_ID);
*/
#if use_MIT_Accurately == 0
	MIT_B.target_position = MIT_B.MIT_TZG - liftoff_R + MIT_Bias_R; // ����ֱ��-125.9�� ����һ����ֵ(liftoff_R)
#endif
	if (isnan(MIT_B.target_position) == 0)
	{
		MIT_B.target_position_end = MIT_B.target_position;
		Buzzer.mode = Zero_times;
	}
	else
	{
		Buzzer.mode = heaps_times;
	}
	//	MIT_B.target_position=MIT_A.MIT_TZG-liftoff_R;
	//			MIT_B.target_position_end=MIT_B.target_position;

	if (MIT_B.target_position < MIT_B.MIT_TSZ - 3)
		MIT_B.target_position_end = MIT_B.MIT_TSZ - 3;

	if (MIT_B.target_position > MIT_B.MIT_TZG + 3)
		MIT_B.target_position_end = MIT_B.MIT_TZG + 3;

	// MIT_B_SPEED.Target=P_PID_bate(&MIT_B_POSITION,MIT_B.target_position,MIT_B.ANGLE_JD);
	//
	MIT_B.target_speed = P_PID_bate_V2(&MIT_B_POSITION, MIT_B.target_position_end, MIT_B.ANGLE_JD);

	//
	// MIT_B.send_to_MIT=P_PID_bate(&MIT_B_SPEED,MIT_B_SPEED.Target,MIT_B.SPEED_JD)/10.0f*send_to_MIT_damping;

	// if(MIT_B.ANGLE_JD>MIT_B.MIT_TZG+1)MIT_B.send_to_MIT=0;
	//
	// if(MIT_B.ANGLE_JD<MIT_B.MIT_TSZ-1)MIT_B.send_to_MIT=0;
	// ���γ���0.3��ʧ��
	// 94.2  -6
	// MIT_B.send_to_MIT=0;
	MIT_B.kp = MIT_B.kp_temp * send_to_MIT_damping;
	MIT_B.kv = MIT_B.kv_temp * send_to_MIT_damping;

	//	position_HD_text=position_text/Angle_turn_Radian;
	MIT_B.send_to_MIT_position = MIT_B.target_position_end / Angle_turn_Radian;
	MIT_B.send_to_MIT_speed = MIT_B.target_speed / Angle_turn_Radian;
	// CanComm_SendControlPara(0,0,0,0,MIT_B.send_to_MIT,MIT_B_SLAVE_ID);

	CanComm_SendControlPara(MIT_B.send_to_MIT_position, MIT_B.send_to_MIT_speed, MIT_B.kp, MIT_B.kv, 0, MIT_B_SLAVE_ID);
	// CanComm_SendControlPara(position_HD_text,speed_HD_text,kp_text,kv_text,send_to_MIT_text);
}

void MIT_A_controul(void)
{
	// liftoff_R+=DR16.rc.ch3/2200.0f;	//��������
	/*

		MIT_A.target_position=118.4-liftoff_R;//����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)

	if(MIT_A.target_position>113)MIT_A.target_position=113;

	if(MIT_A.target_position<26)MIT_A.target_position=26;


	MIT_A_SPEED.Target=P_PID_bate(&MIT_A_POSITION,MIT_A.target_position,MIT_A.ANGLE_JD);



	MIT_A.send_to_MIT=P_PID_bate(&MIT_A_SPEED,MIT_A_SPEED.Target,MIT_A.SPEED_JD)/10.0f;

	if(MIT_A.ANGLE_JD>118.7)MIT_A.send_to_MIT=0;

	if(MIT_A.ANGLE_JD<18.9)MIT_A.send_to_MIT=0;

	//̧�����19.2 19.6 118.4
	CanComm_SendControlPara(0,0,0,0,MIT_A.send_to_MIT,MIT_A_SLAVE_ID);
	*/
#if use_MIT_Accurately == 0
	MIT_A.target_position = MIT_A.MIT_TZG + liftoff_R + MIT_Bias_R; // ����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)
#endif
	if (isnan(MIT_A.target_position) == 0)
	{
		MIT_A.target_position_end = MIT_A.target_position;
		Buzzer.mode = Zero_times;
	}
	else
	{
		Buzzer.mode = heaps_times;
	}
	//			MIT_A.target_position_end=MIT_A.target_position;

	if (MIT_A.target_position > (MIT_A.MIT_TSZ - 3))
		MIT_A.target_position_end = MIT_A.MIT_TSZ - 3;

	if (MIT_A.target_position < (MIT_A.MIT_TZG + 3))
		MIT_A.target_position_end = MIT_A.MIT_TZG + 3;

	// MIT_A_SPEED.Target=P_PID_bate(&MIT_A_POSITION,MIT_A.target_position,MIT_A.ANGLE_JD);
	MIT_A.target_speed = P_PID_bate_V2(&MIT_A_POSITION, MIT_A.target_position_end, MIT_A.ANGLE_JD);

	// MIT_A.send_to_MIT=P_PID_bate(&MIT_A_SPEED,MIT_A_SPEED.Target,MIT_A.SPEED_JD)/10.0f*send_to_MIT_damping;
	// if(MIT_A.ANGLE_JD>MIT_A.MIT_TSZ+1)MIT_A.send_to_MIT=0;
	// if(MIT_A.ANGLE_JD<MIT_A.MIT_TZG-1)MIT_A.send_to_MIT=0;
	// ̧�����-100.2 -1
	// CanComm_SendControlPara(0,0,0,0,MIT_A.send_to_MIT,MIT_A_SLAVE_ID);

	MIT_A.kp = MIT_A.kp_temp * send_to_MIT_damping;
	MIT_A.kv = MIT_A.kv_temp * send_to_MIT_damping;

	MIT_A.send_to_MIT_position = MIT_A.target_position_end / Angle_turn_Radian;
	MIT_A.send_to_MIT_speed = MIT_A.target_speed / Angle_turn_Radian;

	CanComm_SendControlPara(MIT_A.send_to_MIT_position, MIT_A.send_to_MIT_speed, MIT_A.kp, MIT_A.kv, 0, MIT_A_SLAVE_ID);
}
void MIT_C_controul(void)
{
#if use_MIT_Accurately == 0
	MIT_C.target_position = MIT_C.MIT_TZG + liftoff_L + MIT_Bias_L; // ����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)

#endif
	if (isnan(MIT_C.target_position) == 0)
	{
		MIT_C.target_position_end = MIT_C.target_position;
		Buzzer.mode = Zero_times;
	}
	else
	{
		Buzzer.mode = heaps_times;
	}
	if (MIT_C.target_position > MIT_C.MIT_TSZ - 3)
		MIT_C.target_position_end = MIT_C.MIT_TSZ - 3;

	if (MIT_C.target_position < MIT_C.MIT_TZG + 3)
		MIT_C.target_position_end = MIT_C.MIT_TZG + 3;

	// MIT_C_SPEED.Target=P_PID_bate(&MIT_C_POSITION,MIT_C.target_position,MIT_C.ANGLE_JD);
	MIT_C.target_speed = P_PID_bate_V2(&MIT_C_POSITION, MIT_C.target_position_end, MIT_C.ANGLE_JD);

	// MIT_C.send_to_MIT=P_PID_bate(&MIT_C_SPEED,MIT_C_SPEED.Target,MIT_C.SPEED_JD)/10.0f*send_to_MIT_damping;

	// if(MIT_C.ANGLE_JD>MIT_C.MIT_TSZ+1)MIT_C.send_to_MIT=0;
	//
	// if(MIT_C.ANGLE_JD<MIT_C.MIT_TZG-1)MIT_C.send_to_MIT=0;
	MIT_C.kp = MIT_C.kp_temp * send_to_MIT_damping;
	MIT_C.kv = MIT_C.kv_temp * send_to_MIT_damping;

	MIT_C.send_to_MIT_position = MIT_C.target_position_end / Angle_turn_Radian;
	MIT_C.send_to_MIT_speed = MIT_C.target_speed / Angle_turn_Radian;

	// ̧�����20  0.4
	CanComm_SendControlPara(MIT_C.send_to_MIT_position, MIT_C.send_to_MIT_speed, MIT_C.kp, MIT_C.kv, 0, MIT_C_SLAVE_ID);
}
void MIT_D_controul(void)
{
	// liftoff_R+=DR16.rc.ch3/2200.0f;	//��������
	// liftoff_L+=DR16.rc.ch3/2200.0f;	//��������
	// if(liftoff_L>90)liftoff_L=90;
	//
	// if(liftoff_L<10)liftoff_L=10;
#if use_MIT_Accurately == 0
	MIT_D.target_position = MIT_D.MIT_TZG - liftoff_L + MIT_Bias_L; // ����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)

#endif
	if (isnan(MIT_D.target_position) == 0)
	{
		MIT_D.target_position_end = MIT_D.target_position;
		Buzzer.mode = Zero_times;
	}
	else
	{
		Buzzer.mode = heaps_times;
	}
	//		MIT_D.target_position_end=MIT_D.target_position;

	if (MIT_D.target_position < MIT_D.MIT_TSZ + 3)
		MIT_D.target_position_end = MIT_D.MIT_TSZ + 3;

	if (MIT_D.target_position > MIT_D.MIT_TZG - 3)
		MIT_D.target_position_end = MIT_D.MIT_TZG - 3;

	// MIT_D_SPEED.Target=P_PID_bate(&MIT_D_POSITION,MIT_D.target_position,MIT_D.ANGLE_JD);
	MIT_D.target_speed = P_PID_bate_V2(&MIT_D_POSITION, MIT_D.target_position_end, MIT_D.ANGLE_JD);

	// MIT_D.send_to_MIT=P_PID_bate(&MIT_D_SPEED,MIT_D_SPEED.Target,MIT_D.SPEED_JD)/10.0f*send_to_MIT_damping;
	// if(MIT_D.ANGLE_JD>MIT_D.MIT_TZG+1)MIT_D.send_to_MIT=0;
	//
	// if(MIT_D.ANGLE_JD<MIT_D.MIT_TSZ-1)MIT_D.send_to_MIT=0;

	MIT_D.kp = MIT_D.kp_temp * send_to_MIT_damping;
	MIT_D.kv = MIT_D.kv_temp * send_to_MIT_damping;

	MIT_D.send_to_MIT_position = MIT_D.target_position_end / Angle_turn_Radian;
	MIT_D.send_to_MIT_speed = MIT_D.target_speed / Angle_turn_Radian;

	// ̧�����106.2  7
	CanComm_SendControlPara(MIT_D.send_to_MIT_position, MIT_D.send_to_MIT_speed, MIT_D.kp, MIT_D.kv, 0, MIT_D_SLAVE_ID);
}
int send_to_MIT_L_or_R = 0;
int run_MIT_ENTER_MOTO_MODE_times = 0;
void ALL_MIT_ENTER_MOTO_MODE(void)
{
	uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
	send_to_MIT_L_or_R++;
	if (send_to_MIT_L_or_R % 2 == 0)
	{
		CAN_SendData(&hcan2, CAN_ID_STD, MIT_B_SLAVE_ID, buf);
		//	vTaskDelay(1);
		CAN_SendData(&hcan2, CAN_ID_STD, MIT_A_SLAVE_ID, buf);
		//	vTaskDelay(1);
	}
	else
	{
		CAN_SendData(&hcan2, CAN_ID_STD, MIT_C_SLAVE_ID, buf);
		//		vTaskDelay(1);
		CAN_SendData(&hcan2, CAN_ID_STD, MIT_D_SLAVE_ID, buf);
	}
}
void DISABLE_ALL_MIT(void) // ʧ�����е��
{
	uint8_t buf_1[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
	send_to_MIT_L_or_R++;
	if (send_to_MIT_L_or_R % 2 == 0)
	{
		CAN_SendData(&hcan2, CAN_ID_STD, MIT_B_SLAVE_ID, buf_1);
		MIT_A.TX_TIMES++;
		//		vTaskDelay(1);
		CAN_SendData(&hcan2, CAN_ID_STD, MIT_A_SLAVE_ID, buf_1);
		MIT_B.TX_TIMES++;

		//	vTaskDelay(1);
	}
	else
	{
		CAN_SendData(&hcan2, CAN_ID_STD, MIT_C_SLAVE_ID, buf_1);
		MIT_C.TX_TIMES++;

		//		vTaskDelay(1);
		CAN_SendData(&hcan2, CAN_ID_STD, MIT_D_SLAVE_ID, buf_1);
		MIT_D.TX_TIMES++;
	}
	MIT_B.target_position = MIT_B.ANGLE_JD;
	MIT_A.target_position = MIT_A.ANGLE_JD;
	MIT_C.target_position = MIT_C.ANGLE_JD;
	MIT_D.target_position = MIT_D.ANGLE_JD;
}

void MIT_calibration(void)
{

	MIT_A.MIT_TZG = MIT_A.ANGLE_JD;
	MIT_B.MIT_TZG = MIT_B.ANGLE_JD;
	MIT_C.MIT_TZG = MIT_C.ANGLE_JD;
	MIT_D.MIT_TZG = MIT_D.ANGLE_JD;

	MIT_A.MIT_TSZ = MIT_A.MIT_TZG + 99;
	MIT_B.MIT_TSZ = MIT_B.MIT_TZG - 99;
	MIT_C.MIT_TSZ = MIT_C.MIT_TZG + 99;
	MIT_D.MIT_TSZ = MIT_D.MIT_TZG - 99;
}

/*����һ������
ͬ��һ���ؽڵ����ȥһ���Ƕ�ֵ����Ӧ����һ���ؽڵ������һ���Ƕ�ֵ*/
/*ע��,�ȵ����ҿ���c��R��ȷ��,ǰ�����������෴*/
float MIT_Bias_R = 0; /*���ֵΪ����ʱ������ǰ��������б*/
float MIT_Bias_L = 0; /*���ֵ����ʹ�����ȸ����ȱ���һ��*/
float pitch_kp = 10;  /*pitch��̫������,��Ҫ˥��һ��*/
void get_MIT_tg_angle_for_bais(void)
{
	// MIT_Bias_R=DR16.rc.ch1/-30.0f;/*ң����ֱ�ӿ����Ȳ�����б�Ƕ�*/

	// MIT_Bias_R-=DJIC_IMU.total_pitch*pitch_kp;
	/*�����ı��Ȳ���б�Ƕȸı�����
	MIT_Bias_R=POSITION_v2.result*PITCH_XR_K/33/pitch_kp;
	*/
	MIT_Bias_L = -MIT_Bias_R;
	if (fabs(MIT_Bias_R) > 25) // �����������һ���޷�
	{

		if (MIT_Bias_R > 25)
		{
			MIT_Bias_R = 25;
		}
		else if (MIT_Bias_R < -25)
		{
			MIT_Bias_R = -25;
		}
	}

	if (fabs(MIT_Bias_L) > 25) // �����������һ���޷�
	{

		if (MIT_Bias_L > 25)
		{
			MIT_Bias_L = 25;
		}
		else if (MIT_Bias_L < -25)
		{
			MIT_Bias_L = -25;
		}
	}
}
/*��ظ߶Ⱦ�������*/
Ramp_Struct liftoff_temp; // ��ظ߶�б��
void get_MIT_tg_angle_for_liftoff(void)
{
	/*����ҡ��,�޼����*/
	/*
	liftoff_R+=DR16.rc.ch3/2200.0f;	//��������
*/

	/*����ҡ��,�������*/
	/**/
	static int liftoff_mode = 0;
	static int change_mode = 0; // �Ƿ��л��˵�λ,�л���λ����1
	if (DR16.rc.s_left == 2)
	{
		liftoff_mode = 0;
	}
	if (DR16.rc.ch3 == 0) // ������,�ſ��Ի���һ��
	{
		change_mode = 0;
	}
	else if (DR16.rc.ch3 > 300)
	{
		if (change_mode == 0) // û�л���λ,������ִ�л�������
		{
			if (liftoff_mode < 2) // һ��������λ 0 1 2
				liftoff_mode++;	  // �Ӹߵ�λ

			change_mode = 1; // �л���λ����1
		}
	}
	else if (DR16.rc.ch3 < -300)
	{
		if (change_mode == 0) // û�л���λ,������ִ�л�������
		{
			if (liftoff_mode > 0)
				liftoff_mode--; // ��С��λ

			change_mode = 1; // �л���λ����1
		}
	}
	switch (liftoff_mode)
	{
	case 0:
		liftoff_temp.Target_Value = 5;
		break;
	case 1:
		liftoff_temp.Target_Value = 40;
		break;
	case 2:
		liftoff_temp.Target_Value = 65;
		break;
	default:
		break;
	}
	liftoff_temp.Current_Value = liftoff_R;
	liftoff_temp.Absolute_Max = 90;
	liftoff_R = Ramp_Function(&liftoff_temp);

	if (liftoff_R > 90)
		liftoff_R = 90;
	if (liftoff_R < 1)
		liftoff_R = 1;
	liftoff_L = liftoff_R;
}
