#include "ROW_balance.h"
#include "my_positionPID_bate.h"
#include "math.h"
#include "mit_math.h"

/*
��Ҫ���ܣ�
���ֻ���row��ˮƽ

����ֵ��������row��Ƕ�  ����ʵ���ȳ� ����ʵ���ȳ�

���ֵ����/����Ŀ���ȳ�

�߼�������ʵ���ȳ����϶̵��Ǳߵ���Ŀ��߶�  ʵʱ������һ���ȳ�����row��ˮƽ
���߼������ȳ��ȵ�ƽ��ֵ��Ϊ�߶ȣ�ROW�ᱣ��ƽ���ֱֵ�ӵ��ӵ�Ŀ��y��

���ˣ�Ŀ��Y������������ɣ�Ŀ�����߶ȣ��̶�--б��������+ROW�ᱣ��ƽ���ֵ��PID����-ʱ��ˢ�£�+��Ծ�쳤����ͻ�䣩

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
float body_width=52.3;//������
float real_engine_body_height;//��ʵ����߶�

void row_control()
{
leg_L_change_row=P_PID_bate(&keep_ROW_BALENCE,0,DJIC_IMU.Row);

//if(R_C_Y_NOW<L_C_Y_NOW)
//{
//engine_body_height_R=engine_body_height_tg;//�����Ƕ̵��Ǳߣ�����ǿ�Ƶ���Ŀ��߶�
//engine_body_height_L=engine_body_height_tg+leg_L_change_row;
//	
//}
//else if(R_C_Y_NOW>L_C_Y_NOW)
//{
//engine_body_height_L=engine_body_height_tg;//�����Ƕ̵��Ǳߣ�����ǿ�Ƶ���Ŀ��߶�
//engine_body_height_R=engine_body_height_tg-leg_L_change_row;

//}
engine_body_height_L=leg_L_change_row/2.0f;
engine_body_height_R=-leg_L_change_row/2.0f;;

//engine_body_height_R=14-leg_L_change_row;
//	engine_body_height_L=14;
//	leg_L_change_row=L_C_Y_NOW+sin(DJIC_IMU.Row)*body_width;
//	engine_body_height_R=14;

//	engine_body_height_L=14+leg_L_change_row;

}

//void get_real_engine_body_height()
//{

//}




