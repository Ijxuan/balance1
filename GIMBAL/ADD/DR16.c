/* Includes ------------------------------------------------------------------*/
#include "DR16.h"
#include "DR16_RECIVE.h"

/* Private function declarations ---------------------------------------------*/
float DeadZone_Process(float num, float DZ_min, float DZ_max, float DZ_num);
/* function prototypes -------------------------------------------------------*/

    Key_Typedef Key;                  /*<! 18�����������Ϣ */


/**
 * @brief      Initialize DR16 Class
 * @param[in]  None
 * @retval     None
 */
//DR16_Classdef::DR16_Classdef()
//{
//    Status = CTRL_OFF;    //״̬��ʼ��

//    Key.Press_Flag = 0;
//    Key.Long_Press_Flag = 0;
//    Key.Click_Press_Flag = 0;
//	
//    //ҡ��ֵ��ʼ��
//    DataPack.CH0 = 1024;
//    DataPack.CH1 = 1024;
//    DataPack.CH2 = 1024;
//    DataPack.CH3 = 1024;
//    DataPack.CH4 = 1024;
//}

///**
// * @brief      ��ȡ���ݰ�������
// * @param[in]  captureData:�յ������ݰ�ָ��
// * @return     None.
// */
//void DR16_Classdef::DataCapture(DR16_DataPack_Typedef* captureData)
//{
//    // if(captureData == NULL)
//    // {
//    //     return;
//    // }

//    DataPack = *captureData;

//    /*�������Ӱ�ͨ��ֵ��һ������*/
//    RX_Norm = DeadZone_Process((float)(DataPack.CH0 - 1024), -DR16CH_Filter, DR16CH_Filter, 0);
//    RY_Norm = DeadZone_Process((float)(DataPack.CH1 - 1024), -DR16CH_Filter, DR16CH_Filter, 0);
//    LX_Norm = DeadZone_Process((float)(DataPack.CH2 - 1024), -DR16CH_Filter, DR16CH_Filter, 0);
//    LY_Norm = DeadZone_Process((float)(DataPack.CH3 - 1024), -DR16CH_Filter, DR16CH_Filter, 0);
//    DW_Norm = DeadZone_Process((float)(DataPack.CH4 - 1024), -DR16CH_Filter, DR16CH_Filter, 0);

//    MouseX_Norm = DataPack.Mouse_x;
//    MouseY_Norm = DataPack.Mouse_y;
//    MouseZ_Norm = DataPack.Mouse_z;

//    /*��������*/
//    Key_Process();
//}

/**
 * @brief      �������� Key Process
 * @param[in]  None
 * @return     None
 */
void Key_Process(void)
{
	uint32_t KeyMouse = (uint32_t)DR16.keyBoard.key_code | DR16.mouse.keyLeft << 16 | DR16.mouse.keyRight << 17;// �Ѽ������ı�־λ�ϲ���
	
	 for (int Index = 0; Index < 18; Index++)//����ȫ����λ���������ǵ�״̬��
	{
		if (KeyMouse & (1 << Index))//�жϵ�indexλ�Ƿ�Ϊ1��
		{

			Key.PressTime[Index]++;
			if (Key.PressTime[Index] > TIME_KeyMouse_Press)//���㰴�µ�ʱ�䣬��Ϊ����
			{
				Key.Press_Flag |= 1 << Index;//���øü��ı�־λΪ1
			}

			if (Key.PressTime[Index] > TIME_KeyMouse_LongPress)//�����ж�
			{
				Key.Long_Press_Flag |= 1 << Index;//���ó�����־λ
			}
		}
		else
		{
			if ((Key.PressTime[Index] > TIME_KeyMouse_Press)
				&& (Key.PressTime[Index] < TIME_KeyMouse_LongPress))//ʱ�䴦������֮�䣬Ϊ������
			{
				Key.Click_Press_Flag |= 1 << Index;//���õ�����־λ
			}
			else
			{
				Key.Click_Press_Flag &= ~(1 << Index); //ȡ�����������ü��ı�־λ��Ϊ0
			}

			//�Ѿ��ɿ��������±�־λ�ÿա�
			Key.Press_Flag &= ~(1 << Index);
			Key.Long_Press_Flag &= ~(1 << Index);
			Key.PressTime[Index] = 0;
		}

	}
   
}





/**
 * @brief      ����Getxxx�������ܶ��ǻ�����ݰ��е�Get�������ݵ�ֵ��
 * @param[in]  None
 * @return     Get��������ݵ�ֵ
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
 * @brief      ��һ�����ͨ��0123�����XYZֵ(Left_X_Axis,Right_Y_Axis,balabala)
 * @param[in]  None
 * @retval     -660~660֮���ͨ��ֵ
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
 * @brief      �����ж�ĳ�������Ƿ���,��ϼ������ж�
 * @param[in]  _key ͷ�ļ��к궨���key������_w,_s��
 * @retval     ����Ϊture��û����Ϊfalse
 */
//bool IsKeyPress(KeyList_e KeyMouse, KeyPress_Type_e Action)
//{
//   uint8_t action = 0;
//	switch (Action)
//	{
//	case CLICK: //����
//		action = ((Key.Click_Press_Flag >> KeyMouse) & 1);
//		break;
//	case PRESS: //����
//		action = ((Key.Press_Flag >> KeyMouse) & 1);
//		break;
//	case LONGPRESS: //����
//		action = ((Key.Long_Press_Flag >> KeyMouse) & 1);
//		break;
//	default:
//		action = 0;
//		break;
//	}
//	return action;
//}

/**
 * @brief       �ж�DR16״̬
 * @param[in]   None
 * @return      Status
 */
//DR16Status_Typedef DR16_Classdef::GetStatus(void)
//{
//    return Status;
//}




/**
 * @brief      ��������������������㸽����΢С���
 * @param[in]  num:Ҫ�������; DZ_min,DZ_max:������Χ;DZ_num:����������ʱ���ص�ֵ
 * @return     �����Ľ��
 */
float DeadZone_Process(float num, float DZ_min, float DZ_max, float DZ_num)
{
    //�����������򷵻�����ֵ
    if (num<DZ_max && num>DZ_min)
    {
        return DZ_num;
    }
    else
    {
        return num;
    }
}



