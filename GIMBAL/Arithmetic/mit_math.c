#include "mit_math.h"
#include "arm_math.h"
#include "DR16_RECIVE.h"

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

float angle_fai_3=0;//�ڶ�����ϥ�Ǵ��н�

void mit_math_temp()///*ƽ������������*/
{
if(MIT_A.ANGLE_JD-MIT_A.MIT_TZG>10)
{
angle_fai_1=MIT_A.ANGLE_JD-MIT_A.MIT_TZG-30.5;
Bx=-16.0f*cos(angle_fai_1);
By=	16.0f*sin(angle_fai_1);
}
if(MIT_B.ANGLE_JD-MIT_B.MIT_TZG>10)
{
angle_fai_2=MIT_B.ANGLE_JD-MIT_B.MIT_TZG-30.5;
Dx=16.0f*cos(angle_fai_2)+20;
Dy=	16.0f*sin(angle_fai_2);
}

A0_TEMP=64.0f*(Dx-Bx);
B0_TEMP=64.0f*(Dy-By);
C0_TEMP=(Dx-Bx)*(Dx-Bx)+(Dy-By)*(Dy-By);


angle_fai_3=2*atan((B0_TEMP+sqrt(A0_TEMP*A0_TEMP+B0_TEMP*B0_TEMP-C0_TEMP*C0_TEMP))/(A0_TEMP+C0_TEMP));
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

//MIT_B_tg_angle_for_IS=180-angle_fai_2_JD+29.54;
//	if(angle_fai_2_JD<0)
//	{
//MIT_B_tg_angle_for_IS=29.54-180-angle_fai_2_JD;
//	}		
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


void Accurately_contrul_text(void)///*ͨ��ƽ�������������Ŀ��ǶȾ�ȷ���Ʋ���*/
{
	
L_X=10+DR16.rc.ch2/66;
L_Y=25.33+DR16.rc.ch3/66;
	
R_X=10-DR16.rc.ch2/66;
R_Y=25.33+DR16.rc.ch3/66;

}









