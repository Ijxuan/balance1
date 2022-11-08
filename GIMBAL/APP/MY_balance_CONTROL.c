#include "MY_balance_CONTROL.h"
#include "DR16_RECIVE.h"
#include "M3508.h"
#include "my_positionPID_bate.h"

void balance_control(void)
{
if(DR16.rc.s_left==3)
{
    if(DR16.rc.s_right==2)
{

tire_L_TARGE_speed=DR16.rc.ch1*10+DR16.rc.ch0*4;
tire_R_TARGE_speed=DR16.rc.ch1*-10*1.45f+DR16.rc.ch0*4*1.45f;
	/*	
tire_L_TARGE_speed=DR16.rc.ch1*10;
tire_R_TARGE_speed=DR16.rc.ch1*-10*1.45f;*/
send_to_tire_L=P_PID_bate(&TIRE_L_SPEED_pid,tire_L_TARGE_speed,M3508s[3].realSpeed);
send_to_tire_R=P_PID_bate(&TIRE_R_SPEED_pid,tire_R_TARGE_speed,M3508s[2].realSpeed);
}

    if(DR16.rc.s_right==3)
{


tire_L_TARGE_speed=
	P_PID_bate(&BALANCE_P,0,DJIC_IMU.total_pitch)
	+P_PID_bate(&BALANCE_I,0,DJIC_IMU.Gyro_y);
tire_R_TARGE_speed=-1*tire_L_TARGE_speed*1.45f;
	send_to_tire_L=P_PID_bate(&TIRE_L_SPEED_pid,tire_L_TARGE_speed,M3508s[3].realSpeed);
send_to_tire_R=P_PID_bate(&TIRE_R_SPEED_pid,tire_R_TARGE_speed,M3508s[2].realSpeed);
}

}


}









