
#ifndef _DR16_H_
#define _DR16_H_

//#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
//#include <stdint.h>
//#include <stddef.h>
#include "stdint.h"
#include "stdbool.h"

/* Private macros ------------------------------------------------------------*/
#define DR16_DBUS_PACKSIZE 18u //接收机包大小

#define DR16_ROCKER_MAXVALUE 660 //遥控摇杆最大值

#define KEYMOUSE_AMOUNT 18 //键盘鼠标总和：18个键

#define TIME_KeyMouse_Press 1 //超过该时间视为 按下
// --- 在两者之间视为 单击
#define TIME_KeyMouse_LongPress 20 //超过该时间视为 长按


/* 键位枚举 */
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
}KeyPress_Type_e; //鼠标键盘（键）事件类型。

/* Private type --------------------------------------------------------------*/
/**
 * @brief DR16数据包内容
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
 * @brief 手柄上面两挡位开关状态
 */
enum SW_Status_Typedef
{
  Lever_NONE = 0,
  Lever_UP = 1,
  Lever_MID = 3,
  Lever_DOWN = 2,
};


/**
 * @brief 按键类型定义
 */
typedef struct 
{
  uint32_t Press_Flag;		   //键鼠按下标志
  uint32_t Click_Press_Flag; //键鼠单击标志
  uint32_t Long_Press_Flag;	 //键鼠长按标志
  uint8_t PressTime[18];     //键鼠按下持续时间
}Key_Typedef;


/**
  @brief 连接状态
*/
#ifndef __DR16Status_DEFINED
#define __DR16Status_DEFINED
enum DR16Status_Typedef 
{
  CTRL_OFF,    //关闭机器
  CTRL_RC,    //遥控模式
  CTRL_PC    //键鼠模式
};
#endif


/* Exported macros -----------------------------------------------------------*/
#define DR16CH_Filter 12         /*<! 线性死区,手柄或鼠标的归一化后的绝对值小于此值时自动视为0 */

/* Exported types ------------------------------------------------------------*/
/* DR16类型 */
//class DR16_Classdef
//{
//private:
//    DR16Status_Typedef Status;   	  	/*<! DR16状态 */
//    DR16_DataPack_Typedef DataPack;   /*<! 数据包 */
    float RX_Norm, RY_Norm, LX_Norm, LY_Norm, DW_Norm, MouseX_Norm, MouseY_Norm, MouseZ_Norm;
                                      /*<! 两个摇杆四个方向与鼠标三个方向速度归一化后的值 */
//    Key_Typedef Key;                  /*<! 18个键的相关信息 */
    void Key_Process(void);           /*<! 按键处理 */

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

    /*归一化后的通道01234、鼠标XYZ值*/
    float Get_RX_Norm(void);
    float Get_RY_Norm(void);
    float Get_LX_Norm(void);
    float Get_LY_Norm(void);
    float Get_DW_Norm(void);
    float Get_MouseX_Norm(void);
    float Get_MouseY_Norm(void);
    float Get_MouseZ_Norm(void);

    /*用于判断某个按键是否按下,按下之后的回调函数*/
    bool IsKeyPress(KeyList_e KeyMouse, KeyPress_Type_e Action);
//    void SetStatus(DR16Status_Typedef para_status);
//    DR16Status_Typedef GetStatus(void);
////};

#endif
//#endif
