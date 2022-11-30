#ifndef MIT_H
#define MIT_H

#include "main.h"
#include "user_can.h"
#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define TEST_MIT_SLAVE_ID 0x01//�����õ��SLAVEid
#define TEST_MIT_MASTER_ID 0x02//�����õ��MASTERid
typedef struct
{


	float position; //MITλ��

	float velocity; //MIT�ٶ�
	float current;  //MIT����

	int16_t turnCount;	//ת����Ȧ��


} MIT_t;
extern MIT_t text_moto;
extern int MIT_MODE_TEXT;



void MIT_MODE(uint8_t MODE);

#endif
