#include "ROW_balance.h"
#include "my_positionPID_bate.h"

/*
��Ҫ���ܣ�
���ֻ���row��ˮƽ

����ֵ��������row��Ƕ�  ����ʵ���ȳ� ����ʵ���ȳ�

���ֵ����/����Ŀ���ȳ�

�߼�������ʵ���ȳ����϶̵��Ǳߵ���Ŀ��߶�  ʵʱ������һ���ȳ�����row��ˮƽ

����̧�ߣ�DJIC_IMU.RowΪ��ֵ
�����쳤��DJIC_IMU.RowΪ��ֵ
*/

/*
R_C_Y_NOW   ����ʵ���ȳ�
L_C_Y_NOW  ����ʵ���ȳ�

engine_body_height_R Ŀ��
engine_body_height_L Ŀ��

DJIC_IMU.Row
*/
float leg_L_change_row;
void row_control()
{

//if(R_C_Y_NOW<L_C_Y_NOW)
//{
//R_C_Y_NOW=14;//�����Ƕ̵��Ǳߣ�����ǿ�Ƶ���Ŀ��߶�
//}
//else if(R_C_Y_NOW>L_C_Y_NOW)
//{
//L_C_Y_NOW=14;//�����Ƕ̵��Ǳߣ�����ǿ�Ƶ���Ŀ��߶�
//}
engine_body_height_R=14;
leg_L_change_row=P_PID_bate(&keep_ROW_BALENCE,0,DJIC_IMU.Row);
engine_body_height_L=14+leg_L_change_row;
	
}







