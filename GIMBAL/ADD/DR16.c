/* Includes ------------------------------------------------------------------*/
#include "DR16.h"
#include "DR16_RECIVE.h"

/* Private function declarations ---------------------------------------------*/
float DeadZone_Process(float num, float DZ_min, float DZ_max, float DZ_num);
/* function prototypes -------------------------------------------------------*/

    Key_Typedef Key;                  /*<! 18个键的相关信息 */


/**
 * @brief      Initialize DR16 Class
 * @param[in]  None
 * @retval     None
 */
//DR16_Classdef::DR16_Classdef()
//{
//    Status = CTRL_OFF;    //状态初始化

//    Key.Press_Flag = 0;
//    Key.Long_Press_Flag = 0;
//    Key.Click_Press_Flag = 0;
//	
//    //摇杆值初始化
//    DataPack.CH0 = 1024;
//    DataPack.CH1 = 1024;
//    DataPack.CH2 = 1024;
//    DataPack.CH3 = 1024;
//    DataPack.CH4 = 1024;
//}

///**
// * @brief      获取数据包函数，
// * @param[in]  captureData:收到的数据包指针
// * @return     None.
// */
//void DR16_Classdef::DataCapture(DR16_DataPack_Typedef* captureData)
//{
//    // if(captureData == NULL)
//    // {
//    //     return;
//    // }

//    DataPack = *captureData;

//    /*各杂七杂八通道值归一化处理*/
//    RX_Norm = DeadZone_Process((float)(DataPack.CH0 - 1024), -DR16CH_Filter, DR16CH_Filter, 0);
//    RY_Norm = DeadZone_Process((float)(DataPack.CH1 - 1024), -DR16CH_Filter, DR16CH_Filter, 0);
//    LX_Norm = DeadZone_Process((float)(DataPack.CH2 - 1024), -DR16CH_Filter, DR16CH_Filter, 0);
//    LY_Norm = DeadZone_Process((float)(DataPack.CH3 - 1024), -DR16CH_Filter, DR16CH_Filter, 0);
//    DW_Norm = DeadZone_Process((float)(DataPack.CH4 - 1024), -DR16CH_Filter, DR16CH_Filter, 0);

//    MouseX_Norm = DataPack.Mouse_x;
//    MouseY_Norm = DataPack.Mouse_y;
//    MouseZ_Norm = DataPack.Mouse_z;

//    /*按键处理*/
//    Key_Process();
//}

/**
 * @brief      按键处理 Key Process
 * @param[in]  None
 * @return     None
 */
void Key_Process(void)
{
	uint32_t KeyMouse = (uint32_t)DR16.keyBoard.key_code | DR16.mouse.keyLeft << 16 | DR16.mouse.keyRight << 17;// 把键盘鼠标的标志位合并。
	
	 for (int Index = 0; Index < 18; Index++)//遍历全部键位，更新他们的状态。
	{
		if (KeyMouse & (1 << Index))//判断第index位是否为1。
		{

			Key.PressTime[Index]++;
			if (Key.PressTime[Index] > TIME_KeyMouse_Press)//满足按下的时间，视为按下
			{
				Key.Press_Flag |= 1 << Index;//设置该键的标志位为1
			}

			if (Key.PressTime[Index] > TIME_KeyMouse_LongPress)//长按判断
			{
				Key.Long_Press_Flag |= 1 << Index;//设置长按标志位
			}
		}
		else
		{
			if ((Key.PressTime[Index] > TIME_KeyMouse_Press)
				&& (Key.PressTime[Index] < TIME_KeyMouse_LongPress))//时间处于两者之间，为单击。
			{
				Key.Click_Press_Flag |= 1 << Index;//设置单击标志位
			}
			else
			{
				Key.Click_Press_Flag &= ~(1 << Index); //取反操作，将该键的标志位设为0
			}

			//已经松开，将按下标志位置空。
			Key.Press_Flag &= ~(1 << Index);
			Key.Long_Press_Flag &= ~(1 << Index);
			Key.PressTime[Index] = 0;
		}

	}
   
}





/**
 * @brief      以下Getxxx函数功能都是获得数据包中的Get后面数据的值。
 * @param[in]  None
 * @return     Get后面的数据的值
 */
//uint64_t Get_CH0(void)
//{
//    return DataPack.CH0;
//}

//uint64_t Get_CH1(void)
//{
//    return DataPack.CH1;
//}

//uint64_t Get_CH2(void)
//{
//    return DataPack.CH2;
//}

//uint64_t Get_CH3(void)
//{
//    return DataPack.CH3;
//}

//uint64_t Get_CH4(void)
//{
//    return DataPack.CH4;
//}

//SW_Status_Typedef Get_S1_L(void)
//{
//    return (SW_Status_Typedef)DataPack.S1_L;
//}

//SW_Status_Typedef Get_S2_R(void)
//{
//    return (SW_Status_Typedef)DataPack.S2_R;
//}


//int64_t DR16_Classdef::Get_MouseX(void)
//{
//    return DataPack.Mouse_x;
//}

//int64_t DR16_Classdef::Get_MouseY(void)
//{
//    return DataPack.Mouse_y;
//}

//int64_t DR16_Classdef::Get_MouseZ(void)
//{
//    return DataPack.Mouse_z;
//}

//uint64_t DR16_Classdef::Get_Mouse_keyL(void)
//{
//    return DataPack.Mouse_keyL;
//}

//uint64_t DR16_Classdef::Get_Mouse_keyR(void)
//{
//    return DataPack.Mouse_keyR;
//}

//uint64_t DR16_Classdef::Get_Key(void)
//{
//    return DataPack.key;
//}


/**
 * @brief      归一化后的通道0123、鼠标XYZ值(Left_X_Axis,Right_Y_Axis,balabala)
 * @param[in]  None
 * @retval     -660~660之间的通道值
 */
//float DR16_Classdef::Get_RX_Norm(void)
//{
//    return RX_Norm;
//}
//float DR16_Classdef::Get_RY_Norm(void)
//{
//    return RY_Norm;
//}
//float DR16_Classdef::Get_LX_Norm(void)
//{
//    return LX_Norm;
//}
//float DR16_Classdef::Get_LY_Norm(void)
//{
//    return LY_Norm;
//}
//float DR16_Classdef::Get_DW_Norm(void)
//{
//    return DW_Norm;
//}
//float DR16_Classdef::Get_MouseX_Norm(void)
//{
//    return MouseX_Norm;
//}
//float DR16_Classdef::Get_MouseY_Norm(void)
//{
//    return MouseY_Norm;
//}
//float DR16_Classdef::Get_MouseZ_Norm(void)
//{
//    return MouseZ_Norm;
//}

/**
 * @brief      用于判断某个按键是否按下,组合键立马判断
 * @param[in]  _key 头文件中宏定义的key键，如_w,_s等
 * @retval     按下为ture，没按下为false
 */
//bool IsKeyPress(KeyList_e KeyMouse, KeyPress_Type_e Action)
//{
//   uint8_t action = 0;
//	switch (Action)
//	{
//	case CLICK: //单击
//		action = ((Key.Click_Press_Flag >> KeyMouse) & 1);
//		break;
//	case PRESS: //按下
//		action = ((Key.Press_Flag >> KeyMouse) & 1);
//		break;
//	case LONGPRESS: //长按
//		action = ((Key.Long_Press_Flag >> KeyMouse) & 1);
//		break;
//	default:
//		action = 0;
//		break;
//	}
//	return action;
//}

/**
 * @brief       判断DR16状态
 * @param[in]   None
 * @return      Status
 */
//DR16Status_Typedef DR16_Classdef::GetStatus(void)
//{
//    return Status;
//}




/**
 * @brief      死区处理，常用于消除零点附近的微小误差
 * @param[in]  num:要处理的数; DZ_min,DZ_max:死区范围;DZ_num:落在死区内时返回的值
 * @return     处理后的结果
 */
float DeadZone_Process(float num, float DZ_min, float DZ_max, float DZ_num)
{
    //若在死区内则返回死区值
    if (num<DZ_max && num>DZ_min)
    {
        return DZ_num;
    }
    else
    {
        return num;
    }
}



