#include "keyBoard_to_vjoy.h"
#include "DR16_RECIVE.h"
#include "User_math.h"

/*��Ҫ���һ�����ܣ�
�����̼�λӳ���һ������ҡ��

��Ҫ�ܹ�������  ��ʧ��

�ܹ�������ҡ�˵�ֵ����λ���л������߹ۿ�
*/
keyBoard_PRESS keyBoard_W; // W���Ľṹ��
keyBoard_PRESS keyBoard_S; // W���Ľṹ��
keyBoard_PRESS keyBoard_A; // W���Ľṹ��
keyBoard_PRESS keyBoard_D; // W���Ľṹ��

vjoy vjoy_TEST;			// ����ҡ�˲���
Ramp_Struct vjoy_ch_WS; // ����תҡ�� б�º���
Ramp_Struct vjoy_ch_AD; // ����תҡ�� б�º���

float rate_add = 0.01;
float rate_decrease = 0.01;

void keyBoard_WASD()
{
	// DR16.keyBoard.press_W;
	if (DR16.keyBoard.press_W == 1)
	{
		keyBoard_W.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_W.Press_TIMES = 0;
		keyBoard_W.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_W.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_W.Press_static = Click_Press; // ����

		if (keyBoard_W.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_W.Press_static = Long_Press; // ����
		}
	}

	if (keyBoard_W.Press_static_last_time == Click_Press &&
		keyBoard_W.Press_static_last_time == Click_Press)
	{
		keyBoard_W.Click_Press_wait_use = 1; // ������������������һ��ʹ�ü���һ
	}
	//////////

	if (DR16.keyBoard.press_S == 1)
	{
		keyBoard_S.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_S.Press_TIMES = 0;
		keyBoard_S.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_S.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_S.Press_static = Click_Press; // ����

		if (keyBoard_S.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_S.Press_static = Long_Press; // ����
		}
	}

	if (keyBoard_S.Press_static_last_time == Click_Press &&
		keyBoard_S.Press_static_last_time == Click_Press)
	{
		keyBoard_S.Click_Press_wait_use = 1; // ������������������һ��ʹ�ü���һ
	}
	//////////

	if (DR16.keyBoard.press_A == 1)
	{
		keyBoard_A.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_A.Press_TIMES = 0;
		keyBoard_A.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_A.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_A.Press_static = Click_Press; // ����

		if (keyBoard_A.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_A.Press_static = Long_Press; // ����
		}
	}
	//////////

	if (DR16.keyBoard.press_D == 1)
	{
		keyBoard_D.Press_TIMES++; // �������µ�ʱ��
	}
	else
	{
		keyBoard_D.Press_TIMES = 0;
		keyBoard_D.Press_static = No_Press; // �ɿ�
	}
	if (keyBoard_D.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_D.Press_static = Click_Press; // ����

		if (keyBoard_D.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_D.Press_static = Long_Press; // ����
		}
	}
	//////////

	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////

	if (keyBoard_W.Press_static == No_Press && keyBoard_S.Press_static == No_Press) // ws���ɿ�
	{
		vjoy_ch_WS.Target_Value = 0;
		vjoy_ch_WS.Rate = 0.5; // ���ٹ��� 2��
	}

	else if (keyBoard_W.Press_static == Long_Press && keyBoard_S.Press_static != Long_Press) // ��W����
	{
		vjoy_ch_WS.Target_Value = 100;
		vjoy_ch_WS.Rate = 0.2;
	}
	else if (keyBoard_W.Press_static != Long_Press && keyBoard_S.Press_static == Long_Press) // ��S����
	{
		vjoy_ch_WS.Target_Value = -100;
		vjoy_ch_WS.Rate = 0.2;
	}
	else if (keyBoard_W.Press_static == Long_Press && keyBoard_S.Press_static == Long_Press) // WS˫���� ���� ����
	{
		vjoy_ch_WS.Target_Value = 0;
		vjoy_ch_WS.Rate = 1;
	}

	vjoy_ch_WS.Absolute_Max = 101;
	vjoy_ch_WS.Current_Value = vjoy_TEST.ch_WS;

	vjoy_TEST.ch_WS = Ramp_Function(&vjoy_ch_WS);

	keyBoard_W.Press_static_last_time = keyBoard_W.Press_static;
	keyBoard_S.Press_static_last_time = keyBoard_S.Press_static;

	/////////////////////////////////////////////////////////////////

	if (keyBoard_A.Press_static == No_Press && keyBoard_D.Press_static == No_Press) // AD���ɿ�
	{
		vjoy_ch_AD.Target_Value = 0;
		vjoy_ch_AD.Rate = 0.5; // ���ٹ��� 2��
	}

	else if (keyBoard_A.Press_static == Long_Press && keyBoard_D.Press_static != Long_Press) // ��A����
	{
		vjoy_ch_AD.Target_Value = -100;
		vjoy_ch_AD.Rate = 0.15;
	}
	else if (keyBoard_A.Press_static != Long_Press && keyBoard_D.Press_static == Long_Press) // ��D����
	{
		vjoy_ch_AD.Target_Value = 100;
		vjoy_ch_AD.Rate = 0.15;
	}
	else if (keyBoard_A.Press_static == Long_Press && keyBoard_D.Press_static == Long_Press) // AD˫���� ���� ����
	{
		vjoy_ch_AD.Target_Value = 0;
		vjoy_ch_AD.Rate = 1;
	}

	vjoy_ch_AD.Absolute_Max = 101;
	vjoy_ch_AD.Current_Value = vjoy_TEST.ch_AD;

	vjoy_TEST.ch_AD = Ramp_Function(&vjoy_ch_AD);
}
