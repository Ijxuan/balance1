#ifndef __keyBoard_to_vjoy_H
#define __keyBoard_to_vjoy_H

#include "main.h"

#define TIME_KeyMouse_Press 1 // 超过该时间视为 按下。
// 在两者之间视为 单击
#define TIME_KeyMouse_LongPress 20 // 超过该时间视为 长按

typedef enum
{
    Click_Press = 2, // 单击
    Long_Press = 3,  // 长按
    No_Press = 1     // 松开
} Press_static_e;

typedef struct
{

    uint32_t Press_TIMES;           // 按下多久                //键鼠按下标志
    Press_static_e Press_static;    // 键单击标志
    uint8_t Press_static_last_time; // 键标志-上一时刻
    uint8_t Click_Press_wait_use;   // 单击等待使用

} keyBoard_PRESS; // 键的对外输出。

typedef struct
{
    float ch_WS;    // 由W、S两个键组成的通道
    float ch_AD;    // 由A、D两个键组成的通道
    float ch_MOUSE; // 由鼠标两个键组成的通道
    float ch_GB;    // 由G、B两个键组成的通道

} vjoy; // 虚拟摇杆
extern float rate_add;
extern float rate_decrease;
extern vjoy vjoy_TEST;            // 虚拟摇杆测试
extern keyBoard_PRESS keyBoard_W; // W键的结构体

void keyBoard_WASD(void);

#endif
