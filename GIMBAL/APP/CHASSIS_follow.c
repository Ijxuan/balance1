#include "CHASSIS_follow.h"
#include "GM6020.h"


float follow_angle=0;
float follow_angle_real_use=0;
Ramp_Struct CHASSIS_follow_xp; // ���̸���б��
/*������̨�Ƕȼ�����̸���ĽǶ�*/
void YAW_TG_by_gimbal()
{
//follow_angle=(GM6020s[0].readAngle-3417)/8191.0*360;
//	if(GM6020s[0].readAngle>3417&&GM6020s[0].readAngle<7512)
//	{
//	
//	}
	if(GM6020s[0].readAngle>6512)//���⴦��һ�£���Ȼ�����Ż�
	{
	follow_angle=(GM6020s[0].readAngle-8191+2417)*0.0439506775729459;
	}
	else
	{
follow_angle=(GM6020s[0].readAngle-2417)*-0.0439506775729459;	//ֱ�����ӻ�
	}
/**/

CHASSIS_follow_xp.Current_Value=follow_angle_real_use;
CHASSIS_follow_xp.Target_Value=follow_angle;
	follow_angle_real_use=Ramp_Function(&CHASSIS_follow_xp);

}

/*3417-ǹ�ڳ�ǰʱ��6020�Ƕ�*/

/**/









