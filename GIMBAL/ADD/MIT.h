#ifndef MIT_H
#define MIT_H

#include "main.h"
#include "user_can.h"
#include "User_math.h"

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define TEST_MIT_SLAVE_ID 0x09//测试用电机SLAVEid
#define TEST_MIT_MASTER_ID 0x0A//测试用电机MASTERid

#define MIT_A_SLAVE_ID 0x01//测试用电机SLAVEid
#define MIT_A_MASTER_ID 0x02//测试用电机MASTERid

#define MIT_B_SLAVE_ID 0x03//测试用电机SLAVEid
#define MIT_B_MASTER_ID 0x04//测试用电机MASTERid

#define MIT_C_SLAVE_ID 0x05//测试用电机SLAVEid
#define MIT_C_MASTER_ID 0x06//测试用电机MASTERid

#define MIT_D_SLAVE_ID 0x07//测试用电机SLAVEid
#define MIT_D_MASTER_ID 0x08//测试用电机MASTERid
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
	int MIT_SPEED_TEMP;//速度中间变量
	
	float current_end;  //MIT最终电流
		float ANGLE_JD; //角度 
			float SPEED_JD; //角度 

int RC_TIMES;//接收次数
int TX_TIMES;//发送次数
	
float send_to_MIT;//发送给电机的值
float	target_position;//目标角度
uint8_t MIT_RAW_DATA[8];//接收到的电机发来的原始数据

	float MIT_TZG;//抬最高
	float MIT_TSZ;//腿伸直
} MIT_t;




extern MIT_t text_moto;
extern MIT_t MIT_A;
extern MIT_t MIT_B;
extern MIT_t MIT_C;
extern MIT_t MIT_D;


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
extern float target_speed_text_value;//测试用目标速度数值,必须为正值
extern float send_to_MIT_text;//发送给电机的值
extern float target_speed_text;//测试用目标速度

extern int MIT_ANGLE_JD_LAST;//上一时刻电机角度
extern int MIT_SPEED_BY_ANGLE;//根据两次角度之差算出的速度
extern	int i_for_speed;//多久计算一次速度
extern int MIT_ANGLE_JD_CHANGE;//两个时刻电机角度的变化值
extern int MIT_SPEED_BY_ANGLE_TEMP;//根据两次角度之差算出的速度 临时
extern int MIT_ANGLE_JD_LAST_LAST;//上一时刻电机角度
extern int MIT_SPEED_NEW;//临时
extern float target_position_text_PID;//PID测试用目标位置
extern float liftoff_R;//右边离地高度
extern float liftoff_L;//左边离地高度

extern float MAX_OUT;//最大输出
extern Ramp_Struct SEND_TO_MIT_MAX;//
extern float send_to_MIT_damping;//发送给电机的值的衰减

extern int MIT_DISABLE_TIMES;//电机失能时间累计
extern int MIT_ENABLE_TIMES;//电机使能时间累计

extern int run_MIT_ENTER_MOTO_MODE_times;


void MIT_MODE(uint8_t MODE);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void CanComm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t id);
//static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
void MIT_controul(void);
void speed_text_v(void);
void MIT_PID_INIT(void);
void MIT_B_controul(void);
void ALL_MIT_ENTER_MOTO_MODE(void);
void DISABLE_ALL_MIT(void);
void MIT_A_controul(void);
void MIT_C_controul(void);
void MIT_D_controul(void);
void MIT_calibration(void);

#endif
