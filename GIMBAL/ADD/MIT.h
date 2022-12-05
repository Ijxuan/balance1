#ifndef MIT_H
#define MIT_H

#include "main.h"
#include "user_can.h"
#include "User_math.h"

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define TEST_MIT_SLAVE_ID 0x01//测试用电机SLAVEid
#define TEST_MIT_MASTER_ID 0x02//测试用电机MASTERid

// #define P_MIN -0.0f//测试电机位置限幅
// #define P_MAX 98.0f
// #define V_MIN -10.0f
// #define V_MAX 10.0f
// #define KP_MIN 0.0f
// #define KP_MAX 500.0f
// #define KD_MIN 0.0f
// #define KD_MAX 5.0f
// #define T_MIN -18.0f
// #define T_MAX 18.0f

#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f        
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

#define Angle_turn_Radian 57.295779513082320876798154814105f
//弧度转角度要用到
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

//#define Angle_turn_Radian_MIT 57.295779513082320876798154814105f

typedef struct
{


	int position; //MIT位置

	uint16_t velocity; //MIT速度
	int current;  //MIT电流
	float position_end; //MIT最终位置

	float velocity_end; //MIT最终速度
	float current_end;  //MIT最终电流
		float ANGLE_JD; //角度 
			float SPEED_JD; //角度 

	int16_t turnCount;	//转过的圈数
	
uint8_t MIT_RAW_DATA[8];//接收到的电机发来的原始数据

} MIT_t;
extern MIT_t text_moto;
extern int MIT_MODE_TEXT;
extern int16_t sendto_MIT_TEXT;//发送给电机 为1时发 为0时不发
extern float position_text;

extern float position_text;//目标角度
extern float position_HD_text;//目标角度-弧度制

extern float speed_text;//目标速度
extern float speed_HD_text;//目标速度-弧度制

extern float kp_text;//角度系数
extern float kv_text;//速度系数
extern float NJ_text;//目标扭矩
extern Ramp_Struct MIT_P;//目标位置斜坡
extern float position_text_TEMP;//目标角度

void MIT_MODE(uint8_t MODE);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t);
//static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
void MIT_controul(void);

#endif
