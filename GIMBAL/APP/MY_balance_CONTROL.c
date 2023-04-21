#include "MY_balance_CONTROL.h"
#include "DR16_RECIVE.h"
#include "M3508.h"
#include "my_positionPID_bate.h"
#include "LPF.h"
#include "math.h"
#include "keyBoard_to_vjoy.h"



int M3508_speed_new;
float L_R_XS = -0.3f;
float total_pitch_last = 0;
float total_pitch_change = 0;
int L_speed_new;
int R_speed_new;
float enable_total_yaw = 0;
int last_time_rc_right_mode = 0;
int TARGET_speed = 0;
float TARGET_angle_PITCH = 0;
float TARGET_angle_PITCH_BC = -0.55; // 机械平衡角度补偿

float TARGET_angle_YAW = 0;
float TARGET_angle_speed_YAW = 0;
float TARGET_angle_PITCH_MAX = 0.1; // 允许的最大倾角
int TARGET_position = 0;
int TARGET_position_k = 1; // 遥控器控制系数
int TARGET_position_V2 = 0;

float PITCH_XR_K = 0.1; // PITCH轴削弱函数

int DR16_rc_ch1_last;
int DW_FOR_ZX = 0;

int target_speed_by_position = 0;

Ramp_Struct ZX; // 自旋斜坡

int TARGET_speed_RC = 0; // 遥控速度

void balance_control(void)
{
	//    DW_FOR_ZX=DR16.rc.ch4_DW;

	ZX.Current_Value = DW_FOR_ZX;
	ZX.Target_Value = DR16.rc.ch4_DW;
	DW_FOR_ZX = Ramp_Function(&ZX);
	TARGET_speed_RC = DR16.rc.ch1 * 15;
	milemeter();

	PIRCH_XR();
	total_pitch_change = DJIC_IMU.total_pitch - total_pitch_last;

	total_pitch_last = DJIC_IMU.total_pitch;

	// L_speed_new=LPF_V1(M3508s[3].realSpeed,LPF_a);
	// R_speed_new=LPF_V1(M3508s[2].realSpeed,LPF_a);

	L_speed_new = LPF_V2(&SPEED_L, M3508s[3].realSpeed);
	R_speed_new = LPF_V2(&SPEED_R, M3508s[2].realSpeed);
	//	M3508_speed_new=LPF_V1(M3508s[3].realSpeed);

	//	if(DR16.rc.s_right==3&&last_time_rc_right_mode==2)//从3到2......
	//	last_time_rc_right_mode=DR16.rc.s_right;
	if (DR16.rc.s_left == 1)
	{
		if (DR16.rc.s_right == 2)
		{
			if ((abs(DR16.mouse.x) >= 1) || abs(vjoy_TEST.ch_AD) > 1)
			{
				TARGET_angle_YAW = TARGET_angle_YAW + DR16.mouse.x / 700.0 + vjoy_TEST.ch_AD / 500.0f;
			}
			TARGET_angle_speed_YAW = P_PID_bate(&change_direction_angle, TARGET_angle_YAW, DJIC_IMU.total_yaw) * 11;

			tire_L_TARGE_speed = vjoy_TEST.ch_WS * 20 + TARGET_angle_speed_YAW;
			tire_R_TARGE_speed = vjoy_TEST.ch_WS * -20 + TARGET_angle_speed_YAW;

			// tire_L_TARGE_speed=vjoy_TEST.ch_WS*20+vjoy_TEST.ch_AD*15+TARGET_angle_speed_YAW;
			// tire_R_TARGE_speed=vjoy_TEST.ch_WS*-20+vjoy_TEST.ch_AD*15+TARGET_angle_speed_YAW;
			/*
		tire_L_TARGE_speed=DR16.rc.ch1*10;
		tire_R_TARGE_speed=DR16.rc.ch1*-10*1.45f;*/
			send_to_tire_L = P_PID_bate(&TIRE_L_SPEED_pid, tire_L_TARGE_speed, M3508s[3].realSpeed);
			send_to_tire_R = P_PID_bate(&TIRE_R_SPEED_pid, tire_R_TARGE_speed, M3508s[2].realSpeed);

			// send_to_tire_L=0;
			// send_to_tire_R=0;
			//	if(abs(DR16.mouse.x)>=1)
			//	{
			//	TARGET_angle_YAW=TARGET_angle_YAW+DR16.mouse.x/700.0;
			//	}
			//	TARGET_angle_speed_YAW=P_PID_bate(&change_direction_angle,TARGET_angle_YAW,DJIC_IMU.total_yaw);
			// send_to_tire_L+=P_PID_bate(&change_direction_speed,TARGET_angle_speed_YAW,DJIC_IMU.Gyro_z);
			// send_to_tire_R+=P_PID_bate(&change_direction_speed,TARGET_angle_speed_YAW,DJIC_IMU.Gyro_z);
		}
	}
	if (DR16.rc.s_left == 3)
	{
		if (DR16.rc.s_right != 3)
		{
			TARGET_position = M3508s[3].totalAngle - M3508s[2].totalAngle;
			enable_total_yaw = DJIC_IMU.total_yaw;
			TARGET_angle_YAW = DJIC_IMU.total_yaw;

			P_PID_Parameter_Clear(&SPEED_P);
			SPEED_P_v2.Integral = 0;
			SPEED_P_v2.result = 0;

			TARGET_position_V2 = milemeter_test.total_mile_truly_use;
			POSITION_v2.result = 0; // 里程计PID输出置0,因为关节电机用到了这个值
		}

		if (DR16.rc.s_right == 2)
		{

			tire_L_TARGE_speed = DR16.rc.ch3 * 10 + DR16.rc.ch2 * 4;
			tire_R_TARGE_speed = DR16.rc.ch3 * -10 + DR16.rc.ch2 * 4;
			/*
		tire_L_TARGE_speed=DR16.rc.ch1*10;
		tire_R_TARGE_speed=DR16.rc.ch1*-10*1.45f;*/
			send_to_tire_L = P_PID_bate(&TIRE_L_SPEED_pid, tire_L_TARGE_speed, M3508s[3].realSpeed);
			send_to_tire_R = P_PID_bate(&TIRE_R_SPEED_pid, tire_R_TARGE_speed, M3508s[2].realSpeed);
		}

		if (DR16.rc.s_right == 10)//PID调试
		{
			TARGET_speed = DR16.rc.ch1;
			//	TARGET_angle_PITCH=DR16.rc.ch1/-6.6*TARGET_angle_PITCH_MAX;//遥控器直接控制倾斜角度
			//	TARGET_position+=DR16.rc.ch1/-6.6*TARGET_position_k;//遥控器控制目标位置
			//	TARGET_angle_PITCH=P_PID_bate(&POSITION,TARGET_position,M3508s[3].totalAngle-M3508s[2].totalAngle);
			if (DR16.rc.ch1 != 0)
			{
				// TARGET_position_V2=milemeter_test.total_mile_truly_use+DR16.rc.ch1*240;//遥控器给定目标位置

				// TARGET_position_V2=milemeter_test.total_mile_truly_use+P_PID_bate(&RC_SPEED_TO_POSITION,TARGET_speed_RC,L_speed_new-R_speed_new);//遥控器给定目标速度,转换成速度
			}

#if YAW_TEXT == 0 // 当启用YAW轴调试时,批量注释直立环 速度环 位置
			send_to_tire_L =
				//	P_PID_bate_V2(&SPEED_P,0,L_speed_new-R_speed_new)
				//	P_PID_bate(&SPEED_P,0,L_speed_new-R_speed_new)

				P_PID_bate(&BALANCE_P, 0 + TARGET_angle_PITCH_BC, DJIC_IMU.total_pitch) + P_PID_bate(&BALANCE_I, 0, DJIC_IMU.Gyro_y);

			P_PID_bate_V2(&POSITION_v2, TARGET_position_V2, milemeter_test.total_mile_truly_use);
			target_speed_by_position = POSITION_v2.result * PITCH_XR_K;
			;
			P_PID_bate_V2(&SPEED_P_v2, target_speed_by_position, L_speed_new - R_speed_new);

			if (DR16.rc.ch1 != 0) // 前进时关闭速度环
			{
				if ((DR16.rc.ch1 > 0 && L_speed_new - R_speed_new > 0) || (DR16.rc.ch1 < 0 && L_speed_new - R_speed_new < 0))
				// 当前速度与目标速度同向  关闭速度环
				{
					P_PID_Parameter_Clear(&SPEED_P);
					SPEED_P_v2.Integral = 0;
					SPEED_P_v2.result = 0;
				}
			}

			if (DR16.rc.ch1 == 0 && DR16_rc_ch1_last != 0)
			{
				TARGET_position_V2 = milemeter_test.total_mile_truly_use; // 记录松手瞬间的位置
			}

			DR16_rc_ch1_last = DR16.rc.ch1;
			if (DR16.rc.ch0 != 0) // 自旋时关闭速度环
			{
				SPEED_P_v2.result = 0;
			}
			if (DW_FOR_ZX != 0) // 小陀螺关闭速度环
			{
				SPEED_P_v2.result = 0;
			}
			/*调试直立环时注释速度环
			send_to_tire_L+=SPEED_P_v2.result;
			*/
			//			if(DR16.rc.ch0!=0)//自旋时关闭位置环
			//	{
			//	POSITION_v2.result=0;
			//	}

			/*位置环是靠改变姿态重心前后移动*/
			//	send_to_tire_L+=POSITION_v2.result*PITCH_XR_K;//*2023PITCH_XR_K的作用是在平衡状态百分百输出,离平衡状态越远,里程计输出越弱

			send_to_tire_R = -send_to_tire_L;
#endif // 启用YAW轴调试

#if YAW_TEXT == 1
			send_to_tire_R = 0; // 清楚上一次计算结果
			send_to_tire_L = 0;
#endif

			if (DR16.rc.ch0 != 0)
			{
				TARGET_angle_YAW = DJIC_IMU.total_yaw + DR16.rc.ch0 / 15.0;
			}
			TARGET_angle_speed_YAW = P_PID_bate(&change_direction_angle, TARGET_angle_YAW, DJIC_IMU.total_yaw);
			if (DW_FOR_ZX != 0) // 拨轮控制,控制速度
			{
				TARGET_angle_speed_YAW = DW_FOR_ZX;
				TARGET_angle_YAW = DJIC_IMU.total_yaw;
			}
			send_to_tire_L += P_PID_bate(&change_direction_speed, TARGET_angle_speed_YAW, DJIC_IMU.Gyro_z);
			send_to_tire_R += P_PID_bate(&change_direction_speed, TARGET_angle_speed_YAW, DJIC_IMU.Gyro_z);
			//	-P_PID_bate(&SPEED_P,0,M3508s[2].realSpeed)
			//
			//	-P_PID_bate(&BALANCE_P,0,DJIC_IMU.total_pitch)
			//	-P_PID_bate(&BALANCE_I,0,DJIC_IMU.Gyro_y);

			//	+P_PID_bate(&BALANCE_I,0,DJIC_IMU.Gyro_y);
			/*908
		tire_R_TARGE_speed=-1*tire_L_TARGE_speed*1.45f;
			send_to_tire_L=P_PID_bate(&TIRE_L_SPEED_pid,tire_L_TARGE_speed,M3508s[3].realSpeed);
		send_to_tire_R=P_PID_bate(&TIRE_R_SPEED_pid,tire_R_TARGE_speed,M3508s[2].realSpeed);9
		*/

			// send_to_tire_L=0;
			// send_to_tire_R=0;
		}
	}
}

milemeter_t milemeter_test;
void milemeter(void) // 里程计函数  不但要丝滑,还要足够的精度,不然容易超调  total_mile_by_angle_10
{
	milemeter_test.total_mile_by_turnCount = M3508s[3].turnCount - M3508s[2].turnCount;

	milemeter_test.total_mile_by_angle = M3508s[3].totalAngle - M3508s[2].totalAngle;

	milemeter_test.total_mile_by_angle_10 = M3508s[3].totalAngle / 10 - M3508s[2].totalAngle / 10;

	milemeter_test.total_mile_by_angle_100 = M3508s[3].totalAngle / 100 - M3508s[2].totalAngle / 100;

	milemeter_test.total_mile_by_angle_1000 = M3508s[3].totalAngle / 1000 - M3508s[2].totalAngle / 1000;

	milemeter_test.total_mile_by_angle_4000 = M3508s[3].totalAngle / 4000 - M3508s[2].totalAngle / 4000;

	milemeter_test.total_mile_by_angle_8191 = M3508s[3].totalAngle / 8191 - M3508s[2].totalAngle / 8191;

	// milemeter_test.total_mile_truly_use=milemeter_test.total_mile_by_angle;

	milemeter_test.total_mile_truly_use = LPF_V2(&milemeter_A, milemeter_test.total_mile_by_angle / milemeter_test.K_LQR_use);
	//	milemeter_test.total_mile_LQR_use;
}

float PITCH_ZDJD = 55.0f; // 最低角度
void PIRCH_XR(void)		  // PITCH轴削弱
{

	PITCH_XR_K = 1 - (fabs((double)DJIC_IMU.total_pitch) / PITCH_ZDJD);
}

float new_speed_test;
float new_speed_test_last;
float speed_last_time_m_s;
float add_speed_C_last;
float add_speed_C_total;
float add_speed_C_long_time_ago;

int time_JG=1;
int i_qq=0;
float speed_now_time_m_s;
int i_foe_accel=0;


float last_speed_50_ms[50];
float last_accel_50_ms[50];
float last_accel;
float accel_all;
int speed_num;
int accel_num;
float now_speed,now_accel;
float yuche_speed;
void speed_accel()
{
//取本次电机速度
//用50次前的电机速度叠50次加速度的
//记录本次陀螺仪加速度与电机速度


now_speed = speed_now_time_m_s = 0.00040490766f *( M3508s[2].realSpeed-M3508s[3].realSpeed)/2.0f;
now_accel = DJIC_IMU.add_speed_C;
if(speed_num>49){speed_num=0;}
if(accel_num>49){accel_num=0;}

if((accel_num+1) == 50){accel_all -= last_accel_50_ms[0];}
else{accel_all -= last_accel_50_ms[accel_num+1];}
last_accel_50_ms[accel_num] = (last_accel + now_accel) / 2 * 0.001;
accel_all += last_accel_50_ms[accel_num];

if((speed_num+1) == 50){yuche_speed = last_speed_50_ms[0] + accel_all;}
else{yuche_speed = last_speed_50_ms[speed_num+1] + accel_all;}
last_speed_50_ms[speed_num] = now_speed;
last_accel = now_accel;
accel_num++;
speed_num++;

new_speed_test = yuche_speed;
}

//void speed_accel()
//{
//	i_qq++;
//	speed_now_time_m_s=0.00040490766f *( M3508s[2].realSpeed-M3508s[3].realSpeed)/2.0f;
//	if(i_qq>time_JG)
//	{
////new_speed_test=speed_last_time_m_s+time_JG/1000.0f*(DJIC_IMU.add_speed_C+add_speed_C_last)/2.0f;
//new_speed_test=speed_last_time_m_s+time_JG/1000.0f*(add_speed_C_total);
//		
//new_speed_test=speed_last_time_m_s+1000.0f*(add_speed_C_total);

//	speed_last_time_m_s=0.00040490766f *( M3508s[2].realSpeed-M3508s[3].realSpeed)/2.0f;
//		add_speed_C_total=0;//累加清零
//		add_speed_C_last=DJIC_IMU.add_speed_C;
//	i_qq=0;
//	}
//	else
//	{
//	add_speed_C_total+=DJIC_IMU.add_speed_C;//累加
//	}
//new_speed_test_last=new_speed_test;
//}



//#define N 10    // 采样窗口长度
//#define ALPHA 0.7   // 时间加权系数

//float read_speed() {    // 使用实际数据源读取速度
//    // 实现具体的速度读取逻辑
//    float speed = 0.0;
//    return speed;
//}

//float read_add_speed() {    // 使用实际数据源读取加速度
//    // 实现具体的加速度读取逻辑
//    float add_speed = 0.0;
//    return add_speed;
//}

float time_weighted_average(float *data, float alpha, int n) {    // 时间加权平均
//    float result = 0.0;
//    float w = 1.0;
//    for(int i = 0; i < n; i++) {
//        result += w * data[i];
//        w *= (1 - alpha);
//    }
//    result += w * data[n];
//    return result;
}

//    float speed_data[N + 1] = {0.0};   // 存储速度数据的循环数组
//    float add_speed = 0.0;
//    float speed = 0.0;
void speed_accel_V2() 
	{

//        add_speed = DJIC_IMU.add_speed_C;   // 读取加速度数据
//        speed += add_speed / 1000.0;    // 通过加速度计算速度
//        speed_data[N] = speed;          // 存储当前速度数据
//        speed = time_weighted_average(speed_data, ALPHA, N);    // 计算时间加权平均速度
//        for(int i = 0; i < N; i++) {    // 移除最早的速度数据
//            speed_data[i] = speed_data[i + 1];
//									}
//		
	}




