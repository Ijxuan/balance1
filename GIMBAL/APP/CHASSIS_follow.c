#include "CHASSIS_follow.h"
#include "GM6020.h"


float follow_angle=0;
/*������̨�Ƕȼ�����̸���ĽǶ�*/
void YAW_TG_by_gimbal()
{
//follow_angle=(GM6020s[0].readAngle-3417)/8191.0*360;
//	if(GM6020s[0].readAngle>3417&&GM6020s[0].readAngle<7512)
//	{
//	
//	}
	if(GM6020s[0].readAngle>7512)//���⴦��һ�£���Ȼ�����Ż�
	{
	follow_angle=(GM6020s[0].readAngle-8191+3417)*-0.0439506775729459;
	}
	else
	{
follow_angle=(GM6020s[0].readAngle-3417)*-0.0439506775729459;	//ֱ�����ӻ�
	}
/**/




}

/*3417-ǹ�ڳ�ǰʱ��6020�Ƕ�*/

/**/









