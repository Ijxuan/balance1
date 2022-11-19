#include "MY_balance_CONTROL.h"
#include "DR16_RECIVE.h"
#include "M3508.h"
#include "my_positionPID_bate.h"
#include "LPF.h"
#include "math.h"

int M3508_speed_new;
float L_R_XS=-0.3f;
float total_pitch_last=0;
float total_pitch_change=0;
int L_speed_new;
int R_speed_new;
float enable_total_yaw=0;
int last_time_rc_right_mode=0;
int TARGET_speed=0;
float TARGET_angle_PITCH=0;
float TARGET_angle_PITCH_BC=-0.55;//机械平衡角度补偿

float TARGET_angle_YAW=0;
float TARGET_angle_speed_YAW=0;
float TARGET_angle_PITCH_MAX=0.1;//允许的最大倾角
int TARGET_position=0;
int TARGET_position_k=1;//遥控器控制系数
int TARGET_position_V2=0;

float PITCH_XR_K=0.1;//PITCH轴削弱函数

int DR16_rc_ch1_last;

void balance_control(void)
{
	milemeter();
PIRCH_XR();	
	total_pitch_change=DJIC_IMU.total_pitch-total_pitch_last;
	
	total_pitch_last=DJIC_IMU.total_pitch;
	
//	M3508_speed_new=LPF_V1(M3508s[3].realSpeed);
	
//	if(DR16.rc.s_right==3&&last_time_rc_right_mode==2)//从3到2......
//	last_time_rc_right_mode=DR16.rc.s_right;
	
if(DR16.rc.s_left==3)
{
    if(DR16.rc.s_right!=3)
	{
		TARGET_position=M3508s[3].totalAngle-M3508s[2].totalAngle;
	enable_total_yaw=DJIC_IMU.total_yaw;
	TARGET_angle_YAW=DJIC_IMU.total_yaw;	
		
		P_PID_Parameter_Clear(&SPEED_P);
SPEED_P_v2.Integral=0;
SPEED_P_v2.result=0;		

TARGET_position_V2=milemeter_test.total_mile_truly_use;
		
	}

    if(DR16.rc.s_right==2)
{

tire_L_TARGE_speed=DR16.rc.ch1*10+DR16.rc.ch0*4;
tire_R_TARGE_speed=DR16.rc.ch1*-10+DR16.rc.ch0*4;
	/*	
tire_L_TARGE_speed=DR16.rc.ch1*10;
tire_R_TARGE_speed=DR16.rc.ch1*-10*1.45f;*/
send_to_tire_L=P_PID_bate(&TIRE_L_SPEED_pid,tire_L_TARGE_speed,M3508s[3].realSpeed);
send_to_tire_R=P_PID_bate(&TIRE_R_SPEED_pid,tire_R_TARGE_speed,M3508s[2].realSpeed);
	

}

    if(DR16.rc.s_right==3)
{
	TARGET_speed=DR16.rc.ch1;
//	TARGET_angle_PITCH=DR16.rc.ch1/-6.6*TARGET_angle_PITCH_MAX;//遥控器直接控制倾斜角度
//	TARGET_position+=DR16.rc.ch1/-6.6*TARGET_position_k;//遥控器控制目标位置
//	TARGET_angle_PITCH=P_PID_bate(&POSITION,TARGET_position,M3508s[3].totalAngle-M3508s[2].totalAngle);
	if(DR16.rc.ch1!=0)
	{
TARGET_position_V2=milemeter_test.total_mile_truly_use+DR16.rc.ch1*80;
	}
	if(DR16.rc.ch0!=0)
	{
	TARGET_angle_YAW=DJIC_IMU.total_yaw+DR16.rc.ch0/9.0;
	}	
L_speed_new=LPF_V1(M3508s[3].realSpeed);
R_speed_new=LPF_V1(M3508s[2].realSpeed);

		send_to_tire_L=
//	P_PID_bate_V2(&SPEED_P,0,L_speed_new-R_speed_new)
//	P_PID_bate(&SPEED_P,0,L_speed_new-R_speed_new)
	
	P_PID_bate(&BALANCE_P,0+TARGET_angle_PITCH_BC,DJIC_IMU.total_pitch)
	+P_PID_bate(&BALANCE_I,0,DJIC_IMU.Gyro_y);
	
		P_PID_bate_V2(&SPEED_P_v2,0,L_speed_new-R_speed_new);
	if(DR16.rc.ch1!=0)
	{
	P_PID_Parameter_Clear(&SPEED_P);
SPEED_P_v2.Integral=0;
SPEED_P_v2.result=0;		
	}
		if(DR16.rc.ch1==0&&DR16_rc_ch1_last!=0)
		{
		TARGET_position_V2=milemeter_test.total_mile_truly_use;//记录松手瞬间的位置
		}
DR16_rc_ch1_last=DR16.rc.ch1;
	send_to_tire_L+=SPEED_P_v2.result;
	
	P_PID_bate_V2(&POSITION_v2,TARGET_position_V2,milemeter_test.total_mile_truly_use);
	send_to_tire_L+=POSITION_v2.result*PITCH_XR_K;

	
	
	
send_to_tire_R=	-send_to_tire_L;
	
	TARGET_angle_speed_YAW=P_PID_bate(&change_direction_angle,TARGET_angle_YAW,DJIC_IMU.total_yaw);
send_to_tire_L+=P_PID_bate(&change_direction_speed,TARGET_angle_speed_YAW,DJIC_IMU.Gyro_z);
send_to_tire_R+=P_PID_bate(&change_direction_speed,TARGET_angle_speed_YAW,DJIC_IMU.Gyro_z);
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
}
}


}

milemeter_t milemeter_test;
void milemeter(void)//里程计函数  不但要丝滑,还要足够的精度,不然容易超调
{
milemeter_test.total_mile_by_turnCount=M3508s[3].turnCount-M3508s[2].turnCount;

milemeter_test.total_mile_by_angle=M3508s[3].totalAngle/100-M3508s[2].totalAngle/100;
	
milemeter_test.total_mile_by_angle_1000=M3508s[3].totalAngle/1000-M3508s[2].totalAngle/1000;
	
milemeter_test.total_mile_by_angle_4000=M3508s[3].totalAngle/4000-M3508s[2].totalAngle/4000;
	
milemeter_test.total_mile_by_angle_8191=M3508s[3].totalAngle/8191-M3508s[2].totalAngle/8191;
	
milemeter_test.total_mile_truly_use=milemeter_test.total_mile_by_angle_1000;	
}


float PITCH_ZDJD=55.0f;//最低角度
void PIRCH_XR(void)//PITCH轴削弱
{

PITCH_XR_K=1-(fabs((double)DJIC_IMU.total_pitch)/PITCH_ZDJD);

}




