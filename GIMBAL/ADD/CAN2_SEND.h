#ifndef CAN2_H
#define CAN2_H
#include "main.h"
//#include "BSP_CAN.h"		//二选一
//#include "can.h"//二选一
#include "user_can.h"//二选一

//#include "DR16_RECIVE.h"

//#include "GM6020.h"
//#include "my_positionPID_bate.h"
////#include "DJI_C_IMU.h"
#define test_MIT_ID 0x02//测试用电机id


#define L_B_MIT_ID 0x02//左后电机id
#define L_B_MIT_MASTER_ID 0x10//左后电机发送给主机时用的id

#define L_F_MIT_ID 0x1//左前电机id
#define L_F_MIT_MASTER_ID 0x9//左前电机发送给主机时用id
extern int CAN2_SEND_TIMES;
extern int test_MIT_ID_2;


void CAN2_SEND_TO_MIT(void);

//void cloud_control(void);
//void YAW_PID(void);

//void PITCH_PID(void);
//void imu_angle(void);

#endif






