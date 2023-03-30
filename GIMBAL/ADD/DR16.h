
#ifndef _DR16_H_
#define _DR16_H_

//#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
//#include <stdint.h>
//#include <stddef.h>
#include "stdint.h"
#include "stdbool.h"

/* Private macros ------------------------------------------------------------*/
#define DR16_DBUS_PACKSIZE 18u //���ջ�����С

#define DR16_ROCKER_MAXVALUE 660 //ң��ҡ�����ֵ

#define KEYMOUSE_AMOUNT 18 //��������ܺͣ�18����

#define TIME_KeyMouse_Press 1 //������ʱ����Ϊ ����
// --- ������֮����Ϊ ����
#define TIME_KeyMouse_LongPress 20 //������ʱ����Ϊ ����


/* ��λö�� */
typedef enum
{
  KEY_W = 0,
  KEY_S = 1,
  KEY_A,
  KEY_D,
  KEY_SHIFT,
  KEY_CTRL,
  KEY_Q,
  KEY_E,
  KEY_R,
  KEY_F,
  KEY_G,
  KEY_Z,
  KEY_X,
  KEY_C,
  KEY_V,
  KEY_B,
  MOUSE_L,
  MOUSE_R
}KeyList_e;

typedef enum
{
  CLICK,
  PRESS,
  LONGPRESS
}KeyPress_Type_e; //�����̣������¼����͡�

/* Private type --------------------------------------------------------------*/
/**
 * @brief DR16���ݰ�����
 */
#pragma pack(1)
struct DR16_DataPack_Typedef
{
  uint64_t CH0 : 11;
  uint64_t CH1 : 11;
  uint64_t CH2 : 11;
  uint64_t CH3 : 11;
  uint64_t S2_R : 2;
  uint64_t S1_L : 2;
  int64_t Mouse_x : 16;
  int64_t Mouse_y : 16;
  int64_t Mouse_z : 16;
  uint64_t Mouse_keyL : 8;
  uint64_t Mouse_keyR : 8;
  uint64_t key : 16;
  uint64_t CH4 : 11;
};
#pragma pack()

/**
 * @brief �ֱ���������λ����״̬
 */
enum SW_Status_Typedef
{
  Lever_NONE = 0,
  Lever_UP = 1,
  Lever_MID = 3,
  Lever_DOWN = 2,
};


/**
 * @brief �������Ͷ���
 */
typedef struct 
{
  uint32_t Press_Flag;		   //�����±�־
  uint32_t Click_Press_Flag; //���󵥻���־
  uint32_t Long_Press_Flag;	 //���󳤰���־
  uint8_t PressTime[18];     //�����³���ʱ��
}Key_Typedef;


/**
  @brief ����״̬
*/
#ifndef __DR16Status_DEFINED
#define __DR16Status_DEFINED
enum DR16Status_Typedef 
{
  CTRL_OFF,    //�رջ���
  CTRL_RC,    //ң��ģʽ
  CTRL_PC    //����ģʽ
};
#endif


/* Exported macros -----------------------------------------------------------*/
#define DR16CH_Filter 12         /*<! ��������,�ֱ������Ĺ�һ����ľ���ֵС�ڴ�ֵʱ�Զ���Ϊ0 */

/* Exported types ------------------------------------------------------------*/
/* DR16���� */
//class DR16_Classdef
//{
//private:
//    DR16Status_Typedef Status;   	  	/*<! DR16״̬ */
//    DR16_DataPack_Typedef DataPack;   /*<! ���ݰ� */
    float RX_Norm, RY_Norm, LX_Norm, LY_Norm, DW_Norm, MouseX_Norm, MouseY_Norm, MouseZ_Norm;
                                      /*<! ����ҡ���ĸ�������������������ٶȹ�һ�����ֵ */
//    Key_Typedef Key;                  /*<! 18�����������Ϣ */
    void Key_Process(void);           /*<! �������� */

//public:
    /* Exported function declarations --------------------------------------------*/
//    DR16_Classdef();
//    void DataCapture(DR16_DataPack_Typedef* captureData);

    uint64_t Get_CH0(void);
    uint64_t Get_CH1(void);
    uint64_t Get_CH2(void);
    uint64_t Get_CH3(void);
    uint64_t Get_CH4(void);
//    SW_Status_Typedef Get_S2_R(void);
//    SW_Status_Typedef Get_S1_L(void);
    int64_t Get_MouseX(void);
    int64_t Get_MouseY(void);
    int64_t Get_MouseZ(void);
    uint64_t Get_Mouse_keyL(void);
    uint64_t Get_Mouse_keyR(void);
    uint64_t Get_Key(void);

    /*��һ�����ͨ��01234�����XYZֵ*/
    float Get_RX_Norm(void);
    float Get_RY_Norm(void);
    float Get_LX_Norm(void);
    float Get_LY_Norm(void);
    float Get_DW_Norm(void);
    float Get_MouseX_Norm(void);
    float Get_MouseY_Norm(void);
    float Get_MouseZ_Norm(void);

    /*�����ж�ĳ�������Ƿ���,����֮��Ļص�����*/
    bool IsKeyPress(KeyList_e KeyMouse, KeyPress_Type_e Action);
//    void SetStatus(DR16Status_Typedef para_status);
//    DR16Status_Typedef GetStatus(void);
////};

#endif
//#endif
