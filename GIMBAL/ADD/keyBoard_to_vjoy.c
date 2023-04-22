#include "keyBoard_to_vjoy.h"
#include "DR16_RECIVE.h"
#include "User_math.h"

/*主要完成一个功能：
将键盘键位映射成一个虚拟摇杆

需要能够缓启动  缓失能

能够将虚拟摇杆的值在上位机中绘制曲线观看
*/
keyBoard_PRESS keyBoard_W; // W键的结构体
keyBoard_PRESS keyBoard_S; // W键的结构体
keyBoard_PRESS keyBoard_A; // W键的结构体
keyBoard_PRESS keyBoard_D; // W键的结构体

vjoy vjoy_TEST;			// 虚拟摇杆测试
Ramp_Struct vjoy_ch_WS; // 键盘转摇杆 斜坡函数
Ramp_Struct vjoy_ch_AD; // 键盘转摇杆 斜坡函数

float rate_add = 0.01;
float rate_decrease = 0.01;

void keyBoard_WASD()
{
	// DR16.keyBoard.press_W;
	if (DR16.keyBoard.press_W == 1)
	{
		keyBoard_W.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_W.Press_TIMES = 0;
		keyBoard_W.Press_static = No_Press; // 松开
	}
	if (keyBoard_W.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_W.Press_static = Click_Press; // 单击

		if (keyBoard_W.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_W.Press_static = Long_Press; // 长按
		}
	}

	if (keyBoard_W.Press_static_last_time == Click_Press &&
		keyBoard_W.Press_static_last_time == Click_Press)
	{
		keyBoard_W.Click_Press_wait_use = 1; // 单击次数，单击增加一，使用减少一
	}
	//////////

	if (DR16.keyBoard.press_S == 1)
	{
		keyBoard_S.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_S.Press_TIMES = 0;
		keyBoard_S.Press_static = No_Press; // 松开
	}
	if (keyBoard_S.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_S.Press_static = Click_Press; // 单击

		if (keyBoard_S.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_S.Press_static = Long_Press; // 长按
		}
	}

	if (keyBoard_S.Press_static_last_time == Click_Press &&
		keyBoard_S.Press_static_last_time == Click_Press)
	{
		keyBoard_S.Click_Press_wait_use = 1; // 单击次数，单击增加一，使用减少一
	}
	//////////

	if (DR16.keyBoard.press_A == 1)
	{
		keyBoard_A.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_A.Press_TIMES = 0;
		keyBoard_A.Press_static = No_Press; // 松开
	}
	if (keyBoard_A.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_A.Press_static = Click_Press; // 单击

		if (keyBoard_A.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_A.Press_static = Long_Press; // 长按
		}
	}
	//////////

	if (DR16.keyBoard.press_D == 1)
	{
		keyBoard_D.Press_TIMES++; // 按键按下的时间
	}
	else
	{
		keyBoard_D.Press_TIMES = 0;
		keyBoard_D.Press_static = No_Press; // 松开
	}
	if (keyBoard_D.Press_TIMES > TIME_KeyMouse_Press)
	{
		keyBoard_D.Press_static = Click_Press; // 单击

		if (keyBoard_D.Press_TIMES > TIME_KeyMouse_LongPress)
		{
			keyBoard_D.Press_static = Long_Press; // 长按
		}
	}
	//////////

	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////

	if (keyBoard_W.Press_static == No_Press && keyBoard_S.Press_static == No_Press) // ws都松开
	{
		vjoy_ch_WS.Target_Value = 0;
		vjoy_ch_WS.Rate = 0.5; // 快速归零 2秒
	}

	else if (keyBoard_W.Press_static == Long_Press && keyBoard_S.Press_static != Long_Press) // 仅W长按
	{
		vjoy_ch_WS.Target_Value = 100;
		vjoy_ch_WS.Rate = 0.2;
	}
	else if (keyBoard_W.Press_static != Long_Press && keyBoard_S.Press_static == Long_Press) // 仅S长按
	{
		vjoy_ch_WS.Target_Value = -100;
		vjoy_ch_WS.Rate = 0.2;
	}
	else if (keyBoard_W.Press_static == Long_Press && keyBoard_S.Press_static == Long_Press) // WS双长按 算误触 归零
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

	if (keyBoard_A.Press_static == No_Press && keyBoard_D.Press_static == No_Press) // AD都松开
	{
		vjoy_ch_AD.Target_Value = 0;
		vjoy_ch_AD.Rate = 0.5; // 快速归零 2秒
	}

	else if (keyBoard_A.Press_static == Long_Press && keyBoard_D.Press_static != Long_Press) // 仅A长按
	{
		vjoy_ch_AD.Target_Value = -100;
		vjoy_ch_AD.Rate = 0.15;
	}
	else if (keyBoard_A.Press_static != Long_Press && keyBoard_D.Press_static == Long_Press) // 仅D长按
	{
		vjoy_ch_AD.Target_Value = 100;
		vjoy_ch_AD.Rate = 0.15;
	}
	else if (keyBoard_A.Press_static == Long_Press && keyBoard_D.Press_static == Long_Press) // AD双长按 算误触 归零
	{
		vjoy_ch_AD.Target_Value = 0;
		vjoy_ch_AD.Rate = 1;
	}

	vjoy_ch_AD.Absolute_Max = 101;
	vjoy_ch_AD.Current_Value = vjoy_TEST.ch_AD;

	vjoy_TEST.ch_AD = Ramp_Function(&vjoy_ch_AD);
}
