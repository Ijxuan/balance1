#ifndef CHASSIS_CAN_H
#define CHASSIS_CAN_H

#include "DR16_RECIVE.h"

#include "M3508.h"
#include "my_positionPID_bate.h"
//#include "DJI_C_IMU.h"

void CHASSIS_CONTROUL(void);

void switch_change(void);

void Random_CHASSIS(void);

void Cruise_CHASSIS(void);


uint16_t Get_RandomNumbers_Range(int16_t min,int16_t max);


//����˶��ṹ��
typedef struct 
{
    uint32_t number; //�����
    uint16_t sampling; //ɨ��ʱ��
}Random_t;

extern Random_t RANDOM_CHASSIS;

typedef struct
{
			
	//ֱ����������
	int16_t realValue_AB;
	
	int32_t TargerLine;
	
	int16_t lastValue_AB;
	
	int32_t Counts;
	int32_t totalLine;

	P_PID_t P_PID;
	
}Encoder_t;
extern Encoder_t Chassis_Encoder;
void Get_Encoder_Value(Encoder_t* Chassis_Encoder,TIM_HandleTypeDef* htim_ab);

#define OneLoop_LineNumber (1024*4)//������һȦ



#endif

