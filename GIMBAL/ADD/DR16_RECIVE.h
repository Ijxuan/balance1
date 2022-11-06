#ifndef __DR16RECIVE_H
#define __DR16RECIVE_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>


void usart1_dr16_init(void);
void DR16_Process(uint8_t *pData);
void DR_16hander(UART_HandleTypeDef *huart);
void NM_swj(void);
void NM_swj2(void);

#define DR16BufferNumber 22
#define DR16BufferTruthNumber 18
#define DR16BufferLastNumber 4
//extern UART_HandleTypeDef huart1;

extern uint8_t DR16Buffer[DR16BufferNumber];
extern uint8_t CHASSIS_place[8];

extern uint8_t JSBuffer[8];

#define DR16_GroundInit \
{ \
{0,0,0,0,0,0,0}, \
{0,0,0,0,0}, \
{0}, \
0, \
0, \
&DR16_Process, \
} \

#pragma anon_unions
#pragma pack(1)
typedef struct
{
	struct {
		int16_t ch0;//yaw
		int16_t ch1;//pitch
		int16_t ch2;//left_right
		int16_t ch3;//forward_back
		uint8_t s_left;
		uint8_t s_right;
		int16_t ch4_DW; //拨轮
	}rc;//遥控器接收到的原始值。

	struct {
		int16_t x;
		int16_t y;
		int16_t z;

		uint8_t keyLeft;
		uint8_t keyRight;

	}mouse;

	union {//union联合体用法
		uint16_t key_code;
		struct { //位域的使用
			bool press_W : 1;
			bool press_S : 1;
			bool press_A : 1;
			bool press_D : 1;

			bool press_Shift : 1;
			bool press_Ctrl : 1;
			bool press_Q : 1;
			bool press_E : 1;

			bool press_R : 1;
			bool press_F : 1;
			bool press_G : 1;
			bool press_Z : 1;

			bool press_X : 1;
			bool press_C : 1;
			bool press_V : 1;
			bool press_B : 1;
		};
	}keyBoard;



	uint16_t infoUpdateFrame;	//帧率
	uint8_t offLineFlag;		//设备离线标志

	/*指针函数*/
//	void(*DR16_ReInit)(void);
	void(*DR16_Process)(uint8_t *pData);
//	void(*DR16_Handler)(UART_HandleTypeDef *huart);

} DR16_t;
#pragma pack()

extern DR16_t DR16;

#pragma anon_unions
#pragma pack(1)
/* ID: 0X0207          Byte: 7       实时射击数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[7];
		 struct
		{
		  uint8_t bullet_type;//子弹类型: 1：17mm 弹丸 2：42mm 弹丸
      uint8_t shooter_id;//发射机构 ID
      uint8_t bullet_freq;//子弹射频 单位 Hz
      float bullet_speed;//子弹射速 单位 m/s
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_shoot_data_t;
#pragma pack()
extern ext_shoot_data_t ext_shoot_data;
#pragma anon_unions
#pragma pack(1)
/* ID: 0X0206          Byte: 1       伤害状态数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[1];
		 struct
		{
		  uint8_t armor_id : 4;/*bit 0-3：当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的
                             五个装甲片，其他血量变化类型，该变量数值为 0。*/
      uint8_t hurt_type : 4;/*bit 4-7：血量变化类型,0x0 装甲伤害扣血；
                              0x1 模块掉线扣血；
                              0x2 超射速扣血；
                              0x3 超枪口热量扣血；
                              0x4 超底盘功率扣血；
                              0x5 装甲撞击扣血*/
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_robot_hurt_t;
#pragma pack()
extern ext_robot_hurt_t ext_robot_hurt;
#pragma anon_unions
#pragma pack(1)
/* ID: 0X0201          Byte: 27      机器人状态数据 */
typedef struct
{
	union
	{
		uint8_t dataBuff[27];
		 struct
		{
		  uint8_t robot_id;
      uint8_t robot_level;
      uint16_t remain_HP;//机器人剩余血量
      uint16_t max_HP;//机器人上限血量
      uint16_t shooter_id1_17mm_cooling_rate; //机器人 1 号 17mm 枪口每秒冷却值
      uint16_t shooter_id1_17mm_cooling_limit;//机器人 1 号 17mm 枪口热量上限
      uint16_t shooter_id1_17mm_speed_limit;  //机器人 1 号 17mm 枪口上限速度 单位 m/s
      uint16_t shooter_id2_17mm_cooling_rate;
      uint16_t shooter_id2_17mm_cooling_limit;
      uint16_t shooter_id2_17mm_speed_limit;
      uint16_t shooter_id1_42mm_cooling_rate;
      uint16_t shooter_id1_42mm_cooling_limit;
      uint16_t shooter_id1_42mm_speed_limit;
      uint16_t chassis_power_limit;           //机器人底盘功率限制上限
			/*主控电源输出情况：
       0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出；
       1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
       2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；*/
      uint8_t mains_power_gimbal_output : 1;
      uint8_t mains_power_chassis_output : 1;
      uint8_t mains_power_shooter_output : 1;	
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_status_t;
#pragma pack()
extern ext_game_robot_status_t ext_game_robot_state;

#pragma anon_unions
#pragma pack(1)
/* ID: 0X0202          Byte: 16      实时功率热量数据 */
typedef struct
{
  union
	{
		uint8_t dataBuff[16];
		 struct
		{
			uint16_t chassis_volt; //底盘输出电压 单位 毫伏
      uint16_t chassis_current; //底盘输出电流 单位 毫安
      float chassis_power;//底盘输出功率 单位 W 瓦
      uint16_t chassis_power_buffer;//底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
      uint16_t shooter_id1_17mm_cooling_heat;//枪口热量
      uint16_t shooter_id2_17mm_cooling_heat;
      uint16_t shooter_id1_42mm_cooling_heat;	
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_power_heat_data_t;
#pragma pack()
extern ext_power_heat_data_t ext_power_heat_data;

/* ID: 0x0003     Byte: 32     比赛机器人血量数据 */
#pragma pack(1)
typedef struct
{
  union
	{
		uint8_t dataBuff[32];
		__packed struct
		{
	     uint16_t red_1_robot_HP;//红 1 英雄机器人血量，未上场以及罚下血量为 0
       uint16_t red_2_robot_HP;//红 2 工程机器人血量
       uint16_t red_3_robot_HP;//红 3 步兵机器人血量
       uint16_t red_4_robot_HP;//红 4 步兵机器人血量
       uint16_t red_5_robot_HP;//红 5 步兵机器人血量
       uint16_t red_7_robot_HP;//红 7 步兵机器人血量
       uint16_t red_outpost_HP;//红方前哨战血量
       uint16_t red_base_HP;//红方基地血量
       uint16_t blue_1_robot_HP;
       uint16_t blue_2_robot_HP; 
       uint16_t blue_3_robot_HP; 
       uint16_t blue_4_robot_HP; 
       uint16_t blue_5_robot_HP;
       uint16_t blue_7_robot_HP;
       uint16_t blue_outpost_HP;
       uint16_t blue_base_HP;			
		};
	}data;
	uint8_t InfoUpdataFlag;
}ext_game_robot_HP_t;
#pragma pack()
extern ext_game_robot_HP_t   ext_game_robot_HP;


/* ID: 0x0001    Byte: 11     比赛状态数据 */
#pragma pack(1)

typedef struct
{
  union {
		uint8_t dataBuff[11];
		__packed struct {
			uint8_t game_type : 4;             //比赛类型
			uint8_t game_progress : 4;         //当前比赛阶段
			uint16_t stage_remain_time;        //当前阶段剩余时间  单位s
		};
	}data;
	uint8_t infoUpdateFlag;
}ext_game_status_t;
#pragma pack()
extern ext_game_status_t      ext_game_status;



/**********为了匿名四轴上位机的协议定义的变量****************************/
//cup为小端模式存储，也就是在存储的时候，低位被存在0字节，高位在1字节
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))     //取出int型变量的低字节
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))     //    取存储在此变量下一内存字节的内容，高字节
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


#endif
