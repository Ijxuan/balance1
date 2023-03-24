#include "mit_math.h"
#include "arm_math.h"
#include "DR16_RECIVE.h"
#include "my_positionPID_bate.h"
#include "MY_balance_CONTROL.h"
#include "math.h"

/*ƽ�����������*/
/*ѹ��״̬149.5 ���30.5*/

float A0_TEMP=0;
float B0_TEMP=0;
float C0_TEMP=0;

float A1_TEMP=0;
float B1_TEMP=0;
float C1_TEMP=0;

float Dx=0,Dy=0;
float Bx=0,By=0;
float angle_fai_1=0;//����ϵԭ�㴦����ˮƽ��ļн�(���)
float angle_fai_1_JD=0;//����ϵԭ�㴦����ˮƽ��ļн�(���) �Ƕ���

float angle_fai_2=0;//(20,0)������ˮƽ��ļн�(���)
float angle_fai_2_JD=0;//(20,0)������ˮƽ��ļн�(���) �Ƕ���

float angle_qhq=0;//ǰ����б�ĽǶ�
float angle_qhq_pi=0;//ǰ����б�ĽǶ� ������

float C_x_now;
float C_y_now;

float angle_fai_3=0;//�ڶ�����ϥ�Ǵ��н�


float angle_fai_1_zhen=0;//����ϵԭ�㴦����ˮƽ��ļн�(���) ����
float angle_fai_2_zhen=0;//(20,0)������ˮƽ��ļн�(���) ����
float angle_fai_3_zhen=0;//�ڶ�����ϥ�Ǵ��н� ����
float angle_fai_3_zhen_jd=0;//�ڶ�����ϥ�Ǵ��н� ���� �Ƕ�

float A0_TEMP_zhen=0;//����
float B0_TEMP_zhen=0;//����
float C0_TEMP_zhen=0;//����

/*��ڲ�����
����ϵԭ�㴦����ˮƽ��ļн�(���) ����
(20,0)������ˮƽ��ļн�(���) ����
����ֵ��
C���x��y����ֵ��ָ�������
*/
void mit_math_temp(float fai1,float fai2,float *X_now,float *Y_now)///*ƽ������������*/
{

//angle_fai_1_zhen=MIT_A.ANGLE_JD-MIT_A.MIT_TZG-29.54;
//	angle_fai_1_zhen=60;
	angle_fai_1_zhen=fai1;
Bx=-16.0f*cos(angle_fai_1_zhen/180.0f*PI);
By=	16.0f*sin(angle_fai_1_zhen/180.0f*PI);


//angle_fai_2_zhen=MIT_B.MIT_TZG-MIT_B.ANGLE_JD-29.54;
//	angle_fai_2_zhen=60;fai2
		angle_fai_2_zhen=fai2;

Dx=16.0f*cos(angle_fai_2_zhen/180.0f*PI)+20.0;
Dy=	16.0f*sin(angle_fai_2_zhen/180.0f*PI);


A0_TEMP_zhen=64.0f*(Dx-Bx);
B0_TEMP_zhen=64.0f*(Dy-By);
C0_TEMP_zhen=(Dx-Bx)*(Dx-Bx)+(Dy-By)*(Dy-By);


angle_fai_3_zhen=2.0f*atan((B0_TEMP_zhen+sqrt(A0_TEMP_zhen*A0_TEMP_zhen+B0_TEMP_zhen*B0_TEMP_zhen-C0_TEMP_zhen*C0_TEMP_zhen))/(A0_TEMP_zhen+C0_TEMP_zhen));
angle_fai_3_zhen_jd=angle_fai_3_zhen*180.0f/PI;
	
*X_now=(32.0*cos(angle_fai_3_zhen)-16.0*cos(angle_fai_1_zhen/180.0f*PI));

*Y_now=32.0*sin(angle_fai_3_zhen)+16.0*sin(angle_fai_1_zhen/180.0f*PI);

}



float T1=0;//����ϵԭ�㴦����ˮƽ��ļн�(�ٽ�)
float T2=0;//����ϵԭ�㴦����ˮƽ��ļн�(���)
float MIT_A_tg_angle_for_IS=0;//����ϵԭ�㴦�ؽڵ��Ŀ��Ƕ�
float MIT_B_tg_angle_for_IS=0;//(20,0)���ؽڵ��Ŀ��Ƕ�
float TEMP_SQRT=0;
int sqrt_allow=0;//�ܲ�����ƽ����
/*float A0_TEMP=0;
float B0_TEMP=0;
float C0_TEMP=0;

float A1_TEMP=0;
float B1_TEMP=0;
float C1_TEMP=0;*/
void mit_math_temp_2(float Cx,float Cy)///*ƽ�����������*/
{
	//10,18.65
//Bx=-16.0f*cos(angle_fai_1);
A0_TEMP	=Cx*Cx+Cy*Cy-768;//447.8225
B0_TEMP=-32.0*Cx;//-320
C0_TEMP=-32.0*Cy;//-595.2
T1=(	-1.0*C0_TEMP+sqrt(C0_TEMP*C0_TEMP+B0_TEMP*B0_TEMP-A0_TEMP*A0_TEMP) )/(A0_TEMP-B0_TEMP);//89.12/767.8225=0.1160684923924475
//1.570796325����
	if(C0_TEMP*C0_TEMP+B0_TEMP*B0_TEMP-A0_TEMP*A0_TEMP>0)
	{
	sqrt_allow=1;
	}
	else
	{
	sqrt_allow=0;
	}
	TEMP_SQRT=sqrt(C0_TEMP*C0_TEMP+B0_TEMP*B0_TEMP-A0_TEMP*A0_TEMP);//506.0810690924429
angle_fai_1=2*atan(T1);//6.62061009*2=13.24122018
angle_fai_1_JD=angle_fai_1*180/PI;

//MIT_A_tg_angle_for_IS=(PI-angle_fai_1)*180/PI+29.54;
//MIT_A_tg_angle_for_IS=angle_fai_1_JD+29.54;
MIT_A_tg_angle_for_IS=180-angle_fai_1_JD+29.54;
	if(angle_fai_1_JD<0)
	{
MIT_A_tg_angle_for_IS=29.54-180-angle_fai_1_JD;
	}	
	
	
	
A1_TEMP	=(Cx-20)*(Cx-20)+Cy*Cy-768;
B1_TEMP=-32.0*(Cx-20);
C1_TEMP=-32.0*Cy;
T2=(-1.0*C1_TEMP-sqrt(C1_TEMP*C1_TEMP+B1_TEMP*B1_TEMP-A1_TEMP*A1_TEMP) )/(A1_TEMP-B1_TEMP);
angle_fai_2=2*atan(T2);
	angle_fai_2_JD=angle_fai_2*180/PI;
MIT_B_tg_angle_for_IS=angle_fai_2*180/PI+29.54;

}



void get_tg_angle_by_WLG_IS(void)///*ͨ��ƽ�������������Ŀ��Ƕ�*/
{
	#if use_MIT_Accurately==1
//	MIT_A.target_position=MIT_A.MIT_TZG_ARRIVE+MIT_A_tg_angle_for_IS;//����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)
//	MIT_B.target_position=MIT_B.MIT_TZG_ARRIVE-MIT_B_tg_angle_for_IS;//����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)
//	
//	MIT_C.target_position=MIT_C.MIT_TZG_ARRIVE+MIT_A_tg_angle_for_IS;//����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)
//	MIT_D.target_position=MIT_D.MIT_TZG_ARRIVE-MIT_B_tg_angle_for_IS;//����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)
	MIT_A.target_position=MIT_A.MIT_TZG+MIT_A_tg_angle_for_IS;//����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)
	MIT_B.target_position=MIT_B.MIT_TZG-MIT_B_tg_angle_for_IS;//����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)
	
	MIT_C.target_position=MIT_C.MIT_TZG+MIT_A_tg_angle_for_IS;//����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)
	MIT_D.target_position=MIT_D.MIT_TZG-MIT_B_tg_angle_for_IS;//����ֱ��-1�� ��ȥһ����ֵ(liftoff_R)
	
	#endif

}

float change_focus_damping=0;//Ŀ��Ƕȵ�˥��
float keep_BALENCE_by_MIT_THETA_to_X=0;//����ƽ���õĻ�����ܵļн�  ת����X��Ŀ��λ��

void Accurately_contrul_text(void)///*ͨ��ƽ�������������Ŀ��ǶȾ�ȷ���Ʋ���*/
{
	
 	#if use_MIT_change_focus==0
	/*keep_BALENCE_by_MIT_THETA_to_X*/
MIT_keep_BALENCE();
L_X=10+DR16.rc.ch2/660.0*10.0+keep_BALENCE_by_MIT_THETA_to_X;
L_Y=24.33+DR16.rc.ch3/660.0*10.0;
	
R_X=10-DR16.rc.ch2/660.0*10.0-keep_BALENCE_by_MIT_THETA_to_X;
R_Y=24.33+DR16.rc.ch3/660.0*10.0;
	
angle_qhq_pi=atan(	(10.0f-R_C_X_NOW) /R_C_Y_NOW);
angle_qhq	=angle_qhq_pi*180.0f/PI;
	#endif
/*ң�������� 	
L_X=10+MIT_change_focus.result*PITCH_XR_K;
L_Y=25.33+DR16.rc.ch3/66;
	
R_X=10-MIT_change_focus.result*PITCH_XR_K;
R_Y=25.33+DR16.rc.ch3/66;	
*/	
	if(DR16.rc.s_right==2)
	{
		if(change_focus_damping>0.01)
		{
		change_focus_damping-=0.003;//3.3��˥��
		}
		if(change_focus_damping<=0.01)
		{
		change_focus_damping=0;//˥�����
			MIT_change_focus.result=0;
		}	
	}
	if(DR16.rc.s_right==3)
	{
		if(change_focus_damping<0.95)
		{
		change_focus_damping+=0.001;//10������
		}
		if(change_focus_damping>=0.95)
		{
		change_focus_damping=1;//�������
		}	
	}	
	/*
L_X=10+MIT_change_focus.result*change_focus_damping;
L_Y=25.33+DR16.rc.ch3/66;
	
R_X=10-MIT_change_focus.result*change_focus_damping;
R_Y=25.33+DR16.rc.ch3/66;	

	MIT_change_focus_by_speed.result=0;//����ʱרע�������*/
	#if use_MIT_change_focus==1
L_X=10+MIT_change_focus_by_speed.result*change_focus_damping;
L_Y=25.33+DR16.rc.ch3/66;
	
R_X=10-MIT_change_focus_by_speed.result*change_focus_damping;
R_Y=25.33+DR16.rc.ch3/66;	
	#endif
	
}






float keep_BALENCE_by_MIT_RT=0;
Ramp_Struct MIT_BALENCE_start;
Ramp_Struct MIT_BALENCE_GO_TO_TG;

int banlence_times=0;//���屣��ƽ��ʱ��
int banlence_states_this_imes=0;//��һʱ���Ƿ񱣳���ƽ��
int banlence_states_last_times=0;//��һʱ���Ƿ񱣳���ƽ��

int if_use_Ramp_Function=0;//�Ƿ�ʹ��б�º���

float keep_BALENCE_by_MIT_THETA=0;//����ƽ���õĻ�����ܵļн�



void MIT_keep_BALENCE()
{
if(fabs(DJIC_IMU.total_pitch)<13.0f)//ֻ��ƽ��״̬����Ч
	{
					if(R_Y>=15.0f)
		{	
				P_PID_bate(&keep_BALENCE_by_MIT,0,DJIC_IMU.total_pitch-angle_qhq);
		}
else
{
		keep_BALENCE_by_MIT.result=0;

}
		banlence_states_this_imes=1;
		

		
banlence_times++;
	}
	
if(fabs(DJIC_IMU.total_pitch)>15.0f)//���ײ���ƽ��״̬��
	{
banlence_times=0;
		banlence_states_this_imes=0;
		keep_BALENCE_by_MIT.result=0;
	}

	if(banlence_states_this_imes!=banlence_states_last_times)
	{
	//ֻ�ڳ����ض��ĽǶȲŵ���б�º���
		if_use_Ramp_Function=1;
	}
	
	if(if_use_Ramp_Function==1)
	{
		MIT_BALENCE_start.Target_Value=keep_BALENCE_by_MIT.result;
		
		MIT_BALENCE_start.Current_Value=keep_BALENCE_by_MIT_RT;
		
		keep_BALENCE_by_MIT_RT=Ramp_Function(&MIT_BALENCE_start);	
		
	if(fabs(MIT_BALENCE_start.Target_Value-MIT_BALENCE_start.Current_Value)<0.1)
	{
	if_use_Ramp_Function=0;//Ŀ���뵱ǰС��0.1������ʹ��б�º���
	}
	
	}
	if(fabs(DJIC_IMU.total_pitch)<13.0f&&if_use_Ramp_Function==0)//ֻ��ƽ��״̬����Ч  ���Ѿ�������б�º���
	{
//		if(keep_BALENCE_by_MIT.result<=3.0f)
//		{
//		keep_BALENCE_by_MIT.result=0;
//		}		
		
	keep_BALENCE_by_MIT_RT=keep_BALENCE_by_MIT.result;//ֱ�Ӹ�ֵ
//				MIT_BALENCE_GO_TO_TG.Target_Value=keep_BALENCE_by_MIT.result;

//		MIT_BALENCE_GO_TO_TG.Current_Value=keep_BALENCE_by_MIT_RT;
// keep_BALENCE_by_MIT_RT=Ramp_Function(&MIT_BALENCE_GO_TO_TG);//б�¸�ֵ

		
	}
	keep_BALENCE_by_MIT_THETA_to_X=  sin(keep_BALENCE_by_MIT_RT*PI/180.0f)*R_C_Y_NOW;
//		keep_BALENCE_by_MIT_THETA_to_X=0;

	banlence_states_last_times=banlence_states_this_imes;
}

int cumulate_i=0;
int cumulate_time_ms=5;//�ۻ�ʱ��
float cumulate_angle_change=0;//�ۻ��Ƕȱ仯
float pitch_angle_last_time=0;//�ϴ�pitch�Ƕ�
float pitch_speed_new=0;//�������pitch����ٶ�

float cumulate_angle_change_JD=0;//�ۻ��Ƕȱ仯 �Ƕ���
float pitch_speed_new_JD=0;//�������pitch����ٶ� _�Ƕ���

void update_gyro()
{
//DJIC_IMU.total_pitch;
	cumulate_i++;
if(cumulate_i>=cumulate_time_ms)
{
	cumulate_angle_change=angle_qhq_pi-pitch_angle_last_time;//�Ƕȱ仯��
	cumulate_angle_change_JD=cumulate_angle_change*180.0f/PI;//_�Ƕ���
	
	pitch_speed_new=cumulate_angle_change*1000.0f/cumulate_time_ms;//�������pitch����ٶ�
	pitch_speed_new_JD=pitch_speed_new*180.0f/PI;//_�Ƕ���
	pitch_angle_last_time=angle_qhq_pi;
	cumulate_i=0;
}


}
 int speed_to_angle_time_ms=4;
	float change_angle_1ms=0;
	float change_angle_total_speed=0;
		float change_angle_total_speed_end=0;

	float change_angle_total_angle=0;
	float last_angle_total_angle=0;

void gyro_test()
{
static	int speed_to_angle_i=0;


	change_angle_1ms=INS_gyro[0]*0.001;
	change_angle_total_speed+=pitch_speed_new;
	speed_to_angle_i++;
	if(speed_to_angle_i>=speed_to_angle_time_ms)
	{
		change_angle_total_speed_end=change_angle_total_speed/speed_to_angle_time_ms*speed_to_angle_time_ms/1000.0f;
		change_angle_total_angle=angle_qhq_pi-last_angle_total_angle;
		last_angle_total_angle=angle_qhq_pi;
		speed_to_angle_i=0;
change_angle_total_speed=0;
		
	}

}




