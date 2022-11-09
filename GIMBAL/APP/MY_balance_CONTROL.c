#include "MY_balance_CONTROL.h"
#include "DR16_RECIVE.h"
#include "M3508.h"
#include "my_positionPID_bate.h"
float L_R_XS=-1.0f;
float total_pitch_last=0;
float total_pitch_change=0;

void balance_control(void)
{
	
	total_pitch_change=DJIC_IMU.total_pitch-total_pitch_last;
	
	total_pitch_last=DJIC_IMU.total_pitch;
if(DR16.rc.s_left==3)
{
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


send_to_tire_L=
	P_PID_bate(&SPEED_P,0,M3508s[3].realSpeed-M3508s[2].realSpeed)

	+P_PID_bate(&BALANCE_P,0,DJIC_IMU.total_pitch)
	+P_PID_bate(&BALANCE_I,0,DJIC_IMU.Gyro_y);
send_to_tire_R=	-send_to_tire_L;
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









