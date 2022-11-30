#ifndef MIT_H
#define MIT_H

#include "main.h"
#include "user_can.h"
#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define TEST_MIT_SLAVE_ID 0x01//测试用电机SLAVEid
#define TEST_MIT_MASTER_ID 0x02//测试用电机MASTERid
typedef struct
{


	float position; //MIT位置

	float velocity; //MIT速度
	float current;  //MIT电流

	int16_t turnCount;	//转过的圈数


} MIT_t;
extern MIT_t text_moto;
extern int MIT_MODE_TEXT;



void MIT_MODE(uint8_t MODE);

#endif
