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
						 60, -60);					// //MIT电机 速度环
	P_PID_V2_Init(&MIT_A_POSITION, 4, 0.25, 0, 100, //-0.00001
				  //						  float max_error, float min_error,
				  //                          float alpha,
				  50, -50,
				  0.8, -0.8,
				  350, -350,
				  100, -100); // //MIT电机 位置环

	P_PID_Parameter_Init(&MIT_B_SPEED, 0.8, 0.03, -0.6, 4, //-0.00001
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 555, -555,
						 60, -60);					// //MIT电机 速度环
	P_PID_V2_Init(&MIT_B_POSITION, 4, 0.25, 0, 100, //-0.00001
				  //						  float max_error, float min_error,
				  //                          float alpha,
				  50, -50,
				  0.8, -0.8,
				  350, -350,
				  100, -100); // //MIT电机 位置环

	P_PID_Parameter_Init(&MIT_C_SPEED, 0.8, 0.03, -0.6, 4, //-0.00001
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 555, -555,
						 60, -60);					// //MIT电机 速度环
	P_PID_V2_Init(&MIT_C_POSITION, 4, 0.25, 0, 100, //-0.00001
				  //						  float max_error, float min_error,
				  //                          float alpha,
				  50, -50,
				  0.8, -0.8,
				  350, -350,
				  100, -100); // //MIT电机 位置环

	P_PID_Parameter_Init(&MIT_D_SPEED, 0.8, 0.03, -0.6, 4, //-0.00001
						 //						  float max_error, float min_error,
						 //                          float alpha,
						 555, -555,
						 60, -60);					// //MIT电机 速度环
	P_PID_V2_Init(&MIT_D_POSITION, 4, 0.25, 0, 100, //-0.00001
				  //						  float max_error, float min_error,
				  //                          float alpha,
				  50, -50,
				  0.8, -0.8,
				  350, -350,
				  100, -100); // //MIT电机 位置环

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

	MIT_BALENCE_start.Absolute_Max = 20; // 通过MIT保持机体平衡 斜坡函数极限值
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

float liftoff_R = 10;		// 右边离地高度
float liftoff_L = 10;		// 左边离地高度
float engine_body_height_R; // 右边机体高度
float engine_body_height_L; // 左边机体高度
int16_t sendto_MIT_TEXT = 1;
float send_to_MIT_text = 0; // 发送给电机的值
/**/
int MIT_ANGLE_JD_LAST;		// 上一时刻电机角度
int MIT_ANGLE_JD_LAST_LAST; // 上一时刻电机角度

int MIT_ANGLE_JD_CHANGE; // 两个时刻电机角度的变化值

int MIT_SPEED_BY_ANGLE;		 // 根据两次角度之差算出的速度
int MIT_SPEED_BY_ANGLE_TEMP; // 根据两次角度之差算出的速度 临时

int MIT_SPEED_NEW; // 临时

int i_for_speed = 1; // 多久计算一次速度

int L_OR_R = 0;

float send_to_MIT_damping = 0; // 发送给电机的值的衰减

int MIT_DISABLE_TIMES = 0; // 电机失能时间累计
int MIT_ENABLE_TIMES = 0;  // 电机使能时间累计

float L_C_X_NOW = 0; // 此时此刻左轮的坐标（x轴）
float L_C_Y_NOW = 0; // 此时此刻左轮的坐标（y轴）

float R_C_X_NOW = 0; // 此时此刻右轮的坐标（x轴）
float R_C_Y_NOW = 0; // 此时此刻右轮的坐标（y轴）
/* 把buf中的内容通过CAN接口发送出去 */
static void CanTransmit(uint8_t *buf, uint8_t len, uint32_t id)
{
	CAN_TxHeaderTypeDef TxHead; /**!< can通信发送协议头 */
	uint32_t canTxMailbox;

	if ((buf != NULL) && (len != 0))
	{
		TxHead.StdId = id;		   /* 指定标准标识符，该值在0x00-0x7FF */
		TxHead.IDE = CAN_ID_STD;   /* 指定将要传输消息的标识符类型 */
		TxHead.RTR = CAN_RTR_DATA; /* 指定消息传输帧类型 */
		TxHead.DLC = len;		   /* 指定将要传输的帧长度 */

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
		return; /* 直接退出函数 */
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
  * @brief  Can总线发送控制参数
  * @param
  * @retval

  */
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t, uint32_t id)
{
	uint16_t p, v, kp, kd, t;
	uint8_t buf[8];

	/* 限制输入的参数在定义的范围内 */
	LIMIT_MIN_MAX(f_p, P_MIN, P_MAX);
	LIMIT_MIN_MAX(f_v, V_MIN, V_MAX);
	LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
	LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
	LIMIT_MIN_MAX(f_t, T_MIN, T_MAX);

	/* 根据协议，对float参数进行转换 */
	p = float_to_uint(f_p, P_MIN, P_MAX, 16);
	v = float_to_uint(f_v, V_MIN, V_MAX, 12);
	kp = float_to_uint(f_kp, KP_MIN, KP_MAX, 12);
	kd = float_to_uint(f_kd, KD_MIN, KD_MAX, 12);
	t = float_to_uint(f_t, T_MIN, T_MAX, 12);

	/* 根据传输协议，把数据转换为CAN命令数据字段 */
	buf[0] = p >> 8;
	buf[1] = p & 0xFF;
	buf[2] = v >> 4;
	buf[3] = ((v & 0xF) << 4) | (kp >> 8);
	buf[4] = kp & 0xFF;
	buf[5] = kd >> 4;
	buf[6] = ((kd & 0xF) << 4) | (t >> 8);
	buf[7] = t & 0xff;

	/* 通过CAN接口把buf中的内容发送出去 */
	//    CanTransmit(buf, sizeof(buf));

	CAN_SendData(&hcan2, CAN_ID_STD, id, buf);
}

float position_text = 0;	  // 目标角度
float position_text_TEMP = 0; // 目标角度

float position_HD_text = 0; // 目标角度-弧度制

float speed_text = 0;	 // 目标速度
float speed_HD_text = 0; // 目标速度-弧度制

float kp_text = 0; // 角度系数 5  3
float kv_text = 0; // 速度系数 1  1
float NJ_text = 0; // 目标扭矩

float MAX_OUT = 0; // 最大输出

Ramp_Struct MIT_P; // 目标位置斜坡

Ramp_Struct SEND_TO_MIT_MAX; //

float L_X = 0; // 左x目标位置
float L_Y = 0; // 左Y目标位置
float R_X = 0; // 右x目标位置
float R_Y = 0; // 右Y目标位置

int disable_position;
int DR16_rc_ch1_last_for_mit;

int R_speed_new_FOR_MIT;
int L_speed_new_FOR_MIT;

void MIT_controul(void)
{

	// position_text+=DR16.rc.ch1/1100.0f;
	//	speed_HD_text=DR16.rc.ch3/18.0f;//遥控器速度给弧度的目标值+-35

	// if(position_text_TEMP>-1)position_text_TEMP=-1;
	//
	// if(position_text_TEMP<-90)position_text_TEMP=-90;
	// MIT_P.Target_Value=position_text_TEMP;//斜坡目标值
	//	MIT_P.Current_Value=position_text;//斜坡当前值
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

	//	speed_HD_text=P_PID_bate(&MIT_TEXT,position_HD_text,text_moto.position_end);//用弧度制做PID闭环

	//	speed_text=speed_HD_text*Angle_turn_Radian;//目标速度转角度方便观测
	//
	target_position_text_PID += DR16.rc.ch1 / 1100.0f;

	// MIT_P.Target_Value=target_position_text_PID+DR16.rc.ch1/1100.0f;//斜坡目标值
	//	MIT_P.Current_Value=target_position_text_PID;//斜坡当前值
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
	// MIT_C.MIT_TZG_ARRIVE=MIT_C.ANGLE_JD;//刷新边界值
	// }
	// if(MIT_D.ANGLE_JD>MIT_D.MIT_TZG_ARRIVE)
	//{
	// MIT_D.MIT_TZG_ARRIVE=MIT_D.ANGLE_JD;//刷新边界值
	// }
	// if(MIT_A.ANGLE_JD<MIT_A.MIT_TZG_ARRIVE)
	//{
	// MIT_A.MIT_TZG_ARRIVE=MIT_A.ANGLE_JD;//刷新边界值
	// }
	// if(MIT_B.ANGLE_JD>MIT_B.MIT_TZG_ARRIVE)
	//{
	// MIT_B.MIT_TZG_ARRIVE=MIT_B.ANGLE_JD;//刷新边界值
	// }

	if (DR16.rc.s_right == 2)
	{
		L_X = 10;
		R_X = 10;
		// MIT_change_focus.result=0;
		disable_position = milemeter_test.total_mile_by_angle_100; // 失能时
	}
	else if (DR16.rc.s_right == 3)
	{
		if (DR16.rc.ch1 != 0)
		{
			disable_position = milemeter_test.total_mile_by_angle_100 + DR16.rc.ch1 / 3; // 遥控器给定目标位置

			// disable_position=milemeter_test.total_mile_truly_use+P_PID_bate(&RC_SPEED_TO_POSITION,TARGET_speed_RC,L_speed_new-R_speed_new);//遥控器给定目标速度,转换成速度
		}

		if (DR16.rc.ch1 == 0 && DR16_rc_ch1_last_for_mit != 0)
		{
			disable_position = milemeter_test.total_mile_by_angle_100; // 记录松手瞬间的位置
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

	// get_MIT_tg_angle_for_liftoff();//计算腿离地高度

	// get_MIT_tg_angle_for_bais();//计算腿前后倾斜角度

	Accurately_contrul_text();

	if (L_OR_R % 2 == 0)
	{

		mit_math_temp_2(R_X, R_Y); ///*平面五连杆逆解*/
		get_tg_angle_by_WLG_IS();

		MIT_B_controul();
		MIT_A_controul();
	}
	else
	{
		// liftoff_R+=DR16.rc.ch3/2200.0f;	//左手油门
		// liftoff_L+=DR16.rc.ch3/2200.0f;	//左手油门

		// if(liftoff_L>90)liftoff_L=90;
		// if(liftoff_L<1)liftoff_L=1;
		mit_math_temp_2(L_X, L_Y); ///*平面五连杆逆解*/
		get_tg_angle_by_WLG_IS();

		MIT_C_controul();
		MIT_D_controul();
	}
	/*
	float torque_ref = controller->kp*(controller->p_des - controller->theta_mech) + controller->t_ff + controller->kd*(controller->v_des - controller->dtheta_mech);
	*/
}

int speed_add_or_fall;				 // 速度是增大还是减小; 1是增加 2是减小
float target_speed_text = 0;		 // 测试用目标速度
float target_speed_text_value = 80;	 // 测试用目标速度数值,必须为正值
float target_position_text_PID = -1; // PID测试用目标位置

void speed_text_v(void)
{

	// if(text_moto.ANGLE_JD>-10)//
	//{
	// speed_add_or_fall=2;//开始减小
	// }
	// if(text_moto.ANGLE_JD<-80)//
	//{
	// speed_add_or_fall=1;//开始增大
	// }
	// target_speed_text_value=fabs(target_speed_text_value);
	// if(speed_add_or_fall==1)
	//{
	// target_speed_text=target_speed_text_value;//正速度
	// }
	// if(speed_add_or_fall==2)
	//{
	// target_speed_text=-target_speed_text_value;//负速度
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
liftoff_R+=DR16.rc.ch3/2200.0f;	//左手油门
if(liftoff_R>90)liftoff_R=90;

if(liftoff_R<10)liftoff_R=10;
liftoff_L=liftoff_R;
MIT_B.target_position=liftoff_R-125.9;//腿伸直是-125.9度 增加一个正值(liftoff_R)

if(MIT_B.target_position>-30)MIT_B.target_position=-30;

if(MIT_B.target_position<-120)MIT_B.target_position=-120;


MIT_B_SPEED.Target=P_PID_bate(&MIT_B_POSITION,MIT_B.target_position,MIT_B.ANGLE_JD);



MIT_B.send_to_MIT=P_PID_bate(&MIT_B_SPEED,MIT_B_SPEED.Target,MIT_B.SPEED_JD)/10.0f;

if(MIT_B.ANGLE_JD>-25.1)MIT_B.send_to_MIT=0;

if(MIT_B.ANGLE_JD<-126.2)MIT_B.send_to_MIT=0;
//变形超过0.3度失能
//-25.6  -125.9
CanComm_SendControlPara(0,0,0,0,MIT_B.send_to_MIT,MIT_B_SLAVE_ID);
*/
#if use_MIT_Accurately == 0
	MIT_B.target_position = MIT_B.MIT_TZG - liftoff_R + MIT_Bias_R; // 腿伸直是-125.9度 增加一个正值(liftoff_R)
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
	// 变形超过0.3度失能
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
	// liftoff_R+=DR16.rc.ch3/2200.0f;	//左手油门
	/*

		MIT_A.target_position=118.4-liftoff_R;//腿伸直是-1度 减去一个正值(liftoff_R)

	if(MIT_A.target_position>113)MIT_A.target_position=113;

	if(MIT_A.target_position<26)MIT_A.target_position=26;


	MIT_A_SPEED.Target=P_PID_bate(&MIT_A_POSITION,MIT_A.target_position,MIT_A.ANGLE_JD);



	MIT_A.send_to_MIT=P_PID_bate(&MIT_A_SPEED,MIT_A_SPEED.Target,MIT_A.SPEED_JD)/10.0f;

	if(MIT_A.ANGLE_JD>118.7)MIT_A.send_to_MIT=0;

	if(MIT_A.ANGLE_JD<18.9)MIT_A.send_to_MIT=0;

	//抬最高是19.2 19.6 118.4
	CanComm_SendControlPara(0,0,0,0,MIT_A.send_to_MIT,MIT_A_SLAVE_ID);
	*/
#if use_MIT_Accurately == 0
	MIT_A.target_position = MIT_A.MIT_TZG + liftoff_R + MIT_Bias_R; // 腿伸直是-1度 减去一个正值(liftoff_R)
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
	// 抬最高是-100.2 -1
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
	MIT_C.target_position = MIT_C.MIT_TZG + liftoff_L + MIT_Bias_L; // 腿伸直是-1度 减去一个正值(liftoff_R)

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

	// 抬最高是20  0.4
	CanComm_SendControlPara(MIT_C.send_to_MIT_position, MIT_C.send_to_MIT_speed, MIT_C.kp, MIT_C.kv, 0, MIT_C_SLAVE_ID);
}
void MIT_D_controul(void)
{
	// liftoff_R+=DR16.rc.ch3/2200.0f;	//左手油门
	// liftoff_L+=DR16.rc.ch3/2200.0f;	//左手油门
	// if(liftoff_L>90)liftoff_L=90;
	//
	// if(liftoff_L<10)liftoff_L=10;
#if use_MIT_Accurately == 0
	MIT_D.target_position = MIT_D.MIT_TZG - liftoff_L + MIT_Bias_L; // 腿伸直是-1度 减去一个正值(liftoff_R)

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

	// 抬最高是106.2  7
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
void DISABLE_ALL_MIT(void) // 失能所有电机
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

/*创建一个变量
同侧一个关节电机减去一个角度值，相应地另一个关节电机加上一个角度值*/
/*注意,腿的左右看着c板R标确定,前进方向正好相反*/
float MIT_Bias_R = 0; /*这个值为正的时候腿向前进方向倾斜*/
float MIT_Bias_L = 0; /*这个值可以使得左腿跟右腿保持一致*/
float pitch_kp = 10;  /*pitch轴太灵敏了,需要衰减一下*/
void get_MIT_tg_angle_for_bais(void)
{
	// MIT_Bias_R=DR16.rc.ch1/-30.0f;/*遥控器直接控制腿部的倾斜角度*/

	// MIT_Bias_R-=DJIC_IMU.total_pitch*pitch_kp;
	/*依靠改变腿部倾斜角度改变重心
	MIT_Bias_R=POSITION_v2.result*PITCH_XR_K/33/pitch_kp;
	*/
	MIT_Bias_L = -MIT_Bias_R;
	if (fabs(MIT_Bias_R) > 25) // 对最终输出做一个限幅
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

	if (fabs(MIT_Bias_L) > 25) // 对最终输出做一个限幅
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
/*离地高度决定函数*/
Ramp_Struct liftoff_temp; // 离地高度斜坡
void get_MIT_tg_angle_for_liftoff(void)
{
	/*左手摇杆,无级变高*/
	/*
	liftoff_R+=DR16.rc.ch3/2200.0f;	//左手油门
*/

	/*左手摇杆,换挡变高*/
	/**/
	static int liftoff_mode = 0;
	static int change_mode = 0; // 是否切换了挡位,切换挡位后置1
	if (DR16.rc.s_left == 2)
	{
		liftoff_mode = 0;
	}
	if (DR16.rc.ch3 == 0) // 松手了,才可以换下一档
	{
		change_mode = 0;
	}
	else if (DR16.rc.ch3 > 300)
	{
		if (change_mode == 0) // 没切换挡位,接下来执行换挡操作
		{
			if (liftoff_mode < 2) // 一共几个挡位 0 1 2
				liftoff_mode++;	  // 加高挡位

			change_mode = 1; // 切换挡位后置1
		}
	}
	else if (DR16.rc.ch3 < -300)
	{
		if (change_mode == 0) // 没切换挡位,接下来执行换挡操作
		{
			if (liftoff_mode > 0)
				liftoff_mode--; // 减小挡位

			change_mode = 1; // 切换挡位后置1
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
