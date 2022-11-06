#ifndef CLOUD_CAN_H
#define CLOUD_CAN_H

#include "DR16_RECIVE.h"

#include "GM6020.h"
#include "my_positionPID_bate.h"
//#include "DJI_C_IMU.h"
#include "Vision.h"

#define GM6020_border_near_big 2000
#define GM6020_border_near_small 6000

#define GM6020_R_BIG 4736
//这个范围就继续增大吧
#define GM6020_R_MID 4328
//这个范围就继续减小吧
#define GM6020_R_SMALL 3891
//先右后左  
#define GM6020_L_BIG 7847
//这个范围就继续增大吧
#define GM6020_L_MID 151
//这个范围就继续减小吧
#define GM6020_L_SMALL 500

#define YAW_TEXT 0
#define USE_PITCH_BC 0
//打符角度5820    左3701   7748
#if YAW_TEXT==1


#endif
#define USE_MOTOR_angle 1

void cloud_control(void);

void cloud_control_mode_choose(void);


void YAW_PID(void);

void PITCH_PID(void);
void imu_angle(void);

void scan_cloud(void);

//云台
typedef enum
{
	aoto_scan_mode = 0, //云台扫描
	vision_mode,		//云台 自瞄

} Cloud_Control_mode;
typedef struct
{

bool control_mode_NOW;
bool control_mode_LAST;

} cloud_control_mode;
extern cloud_control_mode cloud_mode;


#pragma pack(1)

typedef struct
{
  union {
		uint8_t dataBuff[8];
		__packed struct {
	int32_t totalLine;
	int32_t NO_USE;

		};
	}data;
	uint8_t infoUpdateFlag;
}Encoder_new_t;
#pragma pack()
extern Encoder_new_t Chassis_Encoder_new;

//typedef struct
//{
//	int32_t totalLine;

//}Encoder_t;

typedef struct
{
int game_state_progress;/*0：未开始比赛；
? 1：准备阶段；
? 2：自检阶段；
? 3：5s 倒计时；
? 4：对战中；
? 5：比赛结算中
	*/
int this_progress_remain_time;/*当前阶段剩余时间，单位 */
	
bool red_outpost_is_live;
bool blue_outpost_is_live;	
bool our_outpost_is_live;
} CHASSIS_KEY;
extern CHASSIS_KEY key_message;
extern		 bool scan_i_PITCH;
extern		 int scan_percent_PITCH;//0到1000,百分比
extern		 int scan_time;
extern	 float YAW_TRAGET_ANGLE_TEMP;
extern float PITCH_TRAGET_ANGLE_TEMP;
extern float PITCH_TRAGET_ANGLE_TEMP_EM;

extern bool simulation_target_yaw_is_stop;

extern		 int scan_percent_YAW;//0到1000,百分比
extern	 float YAW_START_ANGLE;//S扫描开始时YAW轴角度
extern int arrive_targe_angle;
extern  bool our_outpost_is_live;

#endif
//MY_CLOUD_COUNTROUL

